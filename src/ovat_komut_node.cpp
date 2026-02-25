#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <functional>

using namespace std;
//BU KODDA YAYA GEÇİDİ KOMUTU YOKTUR 
class OvatKomutNode : public rclcpp::Node {
public:
    OvatKomutNode() : Node("ovat_komut_node"), 
                      mesafe_(0), 
                      wheelData_(0),
                      last_encoder_val_(0), 
                      is_first_encoder_(true),
                      commandBool_(0), 
                      signalBool_(0),
                      detected_lane_angle_(90.0f), 
                      current_steering_angle_(90.0f),
                      active_task_("serit_takip"),
                      distance_(0)
    {
        // Başlangıç durumu
        komut_tabela_.push_back("baslangic");

        // GÖREV FONKSİYONLARINI EŞLEŞTİR (Constructor'ın içi)
        task_handlers_["sagadon"]      = std::bind(&OvatKomutNode::turnRight, this);
        task_handlers_["soladon"]      = std::bind(&OvatKomutNode::turnLeft, this);
        task_handlers_["dur"]          = std::bind(&OvatKomutNode::stopManeuver, this);
        task_handlers_["kirmizi_isik"] = std::bind(&OvatKomutNode::readTrafficSignRed, this);
        task_handlers_["durak"]        = std::bind(&OvatKomutNode::station, this);
        task_handlers_["solnot"]       = std::bind(&OvatKomutNode::notTurnLeft, this);
        task_handlers_["tasitgiremez"] = std::bind(&OvatKomutNode::tasitGiremez, this);
        task_handlers_["ileri_ve_sol"] = std::bind(&OvatKomutNode::forwardLeftForced, this);
        task_handlers_["ileri_ve_sag"] = std::bind(&OvatKomutNode::forwardRightForced, this);
        task_handlers_["zileri"]       = std::bind(&OvatKomutNode::straight, this);
        task_handlers_["kavsak"]       = std::bind(&OvatKomutNode::intersection2, this);
        task_handlers_["zduba"]        = std::bind(&OvatKomutNode::dubaTask, this);
        task_handlers_["yesil_isik"]   = std::bind(&OvatKomutNode::readTrafficSignGreen, this);
        task_handlers_["sag"]          = std::bind(&OvatKomutNode::shortRightTurn, this);
        task_handlers_["sol"]          = std::bind(&OvatKomutNode::shortLeftTurn, this);
        task_handlers_["sagnot"]       = std::bind(&OvatKomutNode::notTurnRight, this);
     // task_handlers_["ztunel"]       = std::bind(&OvatKomutNode::tunnel, this);
     // task_handlers_["park"]         = std::bind(&OvatKomutNode::parkOne, this);
     // task_handlers_["parkedilemez"] = std::bind(&OvatKomutNode::parkTwo, this);
     // task_handlers_["parknot"]      = std::bind(&OvatKomutNode::parkThree, this);
     // task_handlers_["engelli_park"] = std::bind(&OvatKomutNode::parkFour, this);
        

        // SUBSCRIBERS
        sign_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/traffic_sign", 10, std::bind(&OvatKomutNode::signCallback, this, std::placeholders::_1));

        encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/wheel_turns", 10, std::bind(&OvatKomutNode::encoderCallback, this, std::placeholders::_1));

        lane_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/detected_lane_angle", 10, std::bind(&OvatKomutNode::laneCallback, this, std::placeholders::_1));

        // PUBLISHERS
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lane_angle", 10);
        auto_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/autonomous_mode", 10);

        // KONTROL DÖNGÜSÜ (50ms - 20Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&OvatKomutNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "OVAT Beyni Aktif! Modern Router devrede.");
    }

private:
    // --- DEĞİŞKENLER ---
    vector<string> komut_tabela_;
    int mesafe_;
    int wheelData_; 
    int last_encoder_val_;
    bool is_first_encoder_;
    int commandBool_;
    int signalBool_;
    float detected_lane_angle_; 
    float current_steering_angle_; 
    string active_task_; 
    int distance_; 

    // Görevleri ismine göre çağırmak için Map (Sözlük)
    map<string, std::function<void()>> task_handlers_;

    // Tabela Debounce (4 kere okuma) için değişkenler
    string last_seen_sign_ = "";
    int sign_count_ = 0;

    // --- ROS2 PUBLISHER / SUBSCRIBER / TIMER TANIMLAMALARI (Unutulan Kısım) ---
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sign_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lane_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_mode_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // --- YARDIMCI FONKSİYONLAR ---
    string getLastKomut() {
        if (komut_tabela_.empty()) return "";
        return komut_tabela_.back();
    }

    void appendKomut(string yeni_komut) {
        if (getLastKomut() != yeni_komut) {
            komut_tabela_.push_back(yeni_komut);
            if (komut_tabela_.size() > 10) komut_tabela_.erase(komut_tabela_.begin());
        }
    }

    // --- CALLBACKS ---
    void laneCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        detected_lane_angle_ = msg->data;
    }

    void encoderCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        int current_encoder = msg->data;
        if (is_first_encoder_) {
            last_encoder_val_ = current_encoder;
            is_first_encoder_ = false;
            return;
        }
        wheelData_ = current_encoder - last_encoder_val_;
        last_encoder_val_ = current_encoder;
    }

    void signCallback(const std_msgs::msg::String::SharedPtr msg) {
        string raw_data = msg->data; 

        // Gelen veriyi (Örn: "sagadon,45") virgülden ikiye böl
        size_t comma_idx = raw_data.find(',');
        if (comma_idx == string::npos) return; // Virgül yoksa format hatalıdır, işlem yapma

        string sign = raw_data.substr(0, comma_idx);
        string dist_str = raw_data.substr(comma_idx + 1);

        // Mesafeyi int'e çevir ve global distance_ değişkenine kaydet
        try {
            distance_ = std::stoi(dist_str);
        } catch (...) {
            distance_ = 0; // Hata olursa mesafeyi 0 kabul et
        }

        // --- Eski 4 Kere Okuma (Debounce) Mantığı (Aynen Kalıyor) ---
        if (sign == last_seen_sign_) {
            sign_count_++;
        } else {
            last_seen_sign_ = sign;
            sign_count_ = 1;
        }

        if (sign_count_ >= 4) {
            if (active_task_ == "serit_takip" && task_handlers_.find(sign) != task_handlers_.end()) {
                active_task_ = sign;
                mesafe_ = 0;
                RCLCPP_INFO(this->get_logger(), "YENI GOREV: %s | Mesafe: %d cm", sign.c_str(), distance_);
            }
            sign_count_ = 0; 
            last_seen_sign_ = "";
        }
    }

    // ==========================================================
    // PARKUR GÖREV FONKSİYONLARI 
    // ==========================================================

    void readTrafficSignGreen() {
        if (getLastKomut() != "yesil_isik" && getLastKomut() != "yesil_isik_tamam") {
            appendKomut("yesil_isik");
        }

        if (getLastKomut() == "yesil_isik") {
            if (signalBool_ == 0) {
                // Kırmızıda duran arabayı harekete geçir
                auto_mode_pub_->publish(std_msgs::msg::Bool().set__data(true)); 
                signalBool_ = 1;
            }
            appendKomut("yesil_isik_tamam");
            commandBool_ = 0;
            signalBool_ = 0;
        } 
        else if (getLastKomut() == "yesil_isik_tamam") {
            commandBool_ = 0;
            appendKomut("yesil_no_command");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Yesil Isik! Yola devam ediliyor.");
        }
    }

    void shortRightTurn() {
    if (getLastKomut() != "sag" && getLastKomut() != "keskin_donus" && getLastKomut() != "keskin_sag_tamam") {
        appendKomut("sag");
    }

    if (getLastKomut() == "sag") {
        if (distance_ > 1) {
            current_steering_angle_ = detected_lane_angle_;
            commandBool_ = 1;
        } else {
            appendKomut("keskin_donus");
        }
    } 
    else if (getLastKomut() == "keskin_donus") {
        mesafe_ += wheelData_;
        
        if (mesafe_ <= 8) {
            current_steering_angle_ = 90.0f;
            commandBool_ = 1;
        } else if (mesafe_ > 8 && mesafe_ <= 15) {
            current_steering_angle_ = 165.0f;
            commandBool_ = 1;
        } else if (mesafe_ > 15) {
            // Şeride tam oturana ve araba düzleşene kadar (85-100 derece) sağa dönüşe devam et!
            if (detected_lane_angle_ < 85.0f || detected_lane_angle_ > 100.0f) {
                current_steering_angle_ = 165.0f; // Sağa kırmaya devam!
                commandBool_ = 1;
            } else {
                // Araba düzlendi, direksiyonu şeride bırakabiliriz
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
                appendKomut("keskin_sag_tamam");
            }
        }
    } 
    else if (getLastKomut() == "keskin_sag_tamam") {
        commandBool_ = 0;
        mesafe_ = 0;
        appendKomut("sag_gorev_tamam");
        active_task_ = "serit_takip";
        RCLCPP_INFO(this->get_logger(), "Kisa Sag Donus Gorevi Bitti!");
    }
}

    void shortLeftTurn() {
        if (getLastKomut() != "sol" && getLastKomut() != "tabela_yanindayim" && getLastKomut() != "Mission_Completed") {
            appendKomut("sol");
        }

        if (getLastKomut() == "sol") {
            if (distance_ > 1) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                appendKomut("tabela_yanindayim");
            }
        } 
        else if (getLastKomut() == "tabela_yanindayim") {
            mesafe_ += wheelData_;
            
            if (mesafe_ <= 4) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else if (mesafe_ > 4 && mesafe_ <= 14) {
                current_steering_angle_ = 25.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 14) {
                if (detected_lane_angle_ < 85.0f || detected_lane_angle_ > 100.0f) {
                    current_steering_angle_ = 25.0f;
                    commandBool_ = 1;
                } else {
                    current_steering_angle_ = detected_lane_angle_;
                    commandBool_ = 1;
                    appendKomut("Mission_Completed");
                }
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            commandBool_ = 0;
            mesafe_ = 0;
            appendKomut("lefkomututamam");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Kisa Sol Donus Gorevi Bitti!");
        }
    }

    void notTurnRight() {
        commandBool_ = 0;
        mesafe_ = 0;
        active_task_ = "serit_takip";
        RCLCPP_INFO(this->get_logger(), "Saga Donulmez! Duz devam ediliyor.");
    }
    
    void dubaTask() {
        if (getLastKomut() != "DubaKomut" && getLastKomut() != "DuzDuba" && getLastKomut() != "Mission_Completed") {
            appendKomut("DubaKomut");
        }

        if (getLastKomut() == "DubaKomut") {
            if (distance_ > 1) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                appendKomut("DuzDuba");
            }
        } 
        else if (getLastKomut() == "DuzDuba") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 16) {
                current_steering_angle_ = detected_lane_angle_; // Dubayı geçerken şeridi takip et
                commandBool_ = 1;
            } else if (mesafe_ > 16) {
                appendKomut("Mission_Completed");
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            mesafe_ = 0;
            commandBool_ = 0;
            appendKomut("No_Command_Duba");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Duba Gorevi Bitti!");
        }
    }
/*
    void parkOne() {
        // active_task_ değişkeni o anki tabela ismini tutar (örn: "park")
        if (getLastKomut() != active_task_ && getLastKomut() != "serit_ara" && getLastKomut() != "Mission_Completed") {
            appendKomut(active_task_);
        }

        if (getLastKomut() == active_task_) {
            mesafe_ += wheelData_;
            if (mesafe_ <= 6) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 6) {
                appendKomut("serit_ara");
                mesafe_ = 0;
            }
        } 
        else if (getLastKomut() == "serit_ara") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 4) {
                current_steering_angle_ = detected_lane_angle_; // Şeride otur
                commandBool_ = 1;
            } else if (mesafe_ > 4) {
                appendKomut("Mission_Completed");
                mesafe_ = 0;
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            mesafe_ = 0;
            commandBool_ = 0;
            appendKomut("no_command");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Park Senaryosu 1 Bitti!");
        }
    }

    void parkTwo() {
        if (getLastKomut() != active_task_ && getLastKomut() != "serit_ara" && getLastKomut() != "Mission_Completed") {
            appendKomut(active_task_);
        }

        if (getLastKomut() == active_task_) {
            mesafe_ += wheelData_;
            if (mesafe_ <= 2) {
                current_steering_angle_ = 60.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 2 && mesafe_ <= 4) {
                current_steering_angle_ = 100.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 4 && mesafe_ <= 7) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 7) {
                appendKomut("serit_ara");
                mesafe_ = 0;
            }
        } 
        else if (getLastKomut() == "serit_ara") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 4) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else if (mesafe_ > 4) {
                appendKomut("Mission_Completed");
                mesafe_ = 0;
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            mesafe_ = 0;
            commandBool_ = 0;
            appendKomut("no_command");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Park Senaryosu 2 Bitti!");
        }
    }

    void parkThree() {
        if (getLastKomut() != active_task_ && getLastKomut() != "serit_ara" && getLastKomut() != "Mission_Completed") {
            appendKomut(active_task_);
        }

        if (getLastKomut() == active_task_) {
            mesafe_ += wheelData_;
            if (mesafe_ <= 4) {
                current_steering_angle_ = 40.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 4 && mesafe_ <= 7) {
                current_steering_angle_ = 110.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 7 && mesafe_ <= 10) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 10) {
                appendKomut("serit_ara");
                commandBool_ = 1;
                mesafe_ = 0;
            }
        } 
        else if (getLastKomut() == "serit_ara") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 4) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else if (mesafe_ > 4) {
                appendKomut("Mission_Completed");
                mesafe_ = 0;
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            mesafe_ = 0;
            commandBool_ = 0;
            
            // Python kodunda Park 3 tamamlanınca fren yapılıyor (serCom.brakeSignalSandData)
            RCLCPP_INFO(this->get_logger(), "Park Senaryosu 3 Bitti! Arac durduruluyor.");
            auto_mode_pub_->publish(std_msgs::msg::Bool().set__data(false)); // Fren
            
            appendKomut("no_command");
            active_task_ = "serit_takip"; // İstenirse burada beklemede bırakılabilir
        }
    }

    void parkFour() {
        if (getLastKomut() != active_task_ && getLastKomut() != "Tabela_ara") {
            appendKomut(active_task_);
        }

        if (getLastKomut() == active_task_) {
            mesafe_ += wheelData_;
            if (mesafe_ <= 6) {
                current_steering_angle_ = 30.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 6 && mesafe_ <= 9) {
                current_steering_angle_ = 120.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 9) {
                commandBool_ = 1;
                appendKomut("Tabela_ara");
                mesafe_ = 0; // Mesafe sıfırlanıyor
            }
        } 
        else if (getLastKomut() == "Tabela_ara") {
            commandBool_ = 0;
            mesafe_ = 0;
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Park Senaryosu 4 Bitti! Yeni tabela bekleniyor.");
        }
    }
*/
    void forwardLeftForced() {
        if (getLastKomut() != "ileri_sola_mecburi" && getLastKomut() != "Mission_Completed" && getLastKomut() != "ileri_sola") {
            appendKomut("ileri_sola_mecburi");
        }

        if (getLastKomut() == "ileri_sola_mecburi") {
            if (distance_ > 1) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                appendKomut("ileri_sola");
            }
        } 
        else if (getLastKomut() == "ileri_sola") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 8) {
                current_steering_angle_ = detected_lane_angle_; // O anki şerit neyse koru
                commandBool_ = 1;
            } else if (mesafe_ > 8) {
                appendKomut("Mission_Completed");
                mesafe_ = 0;
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            commandBool_ = 0;
            appendKomut("no_command_forwardLeftForced");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Ileri Sola Mecburi Gorevi Bitti!");
        }
    }
/*
    void tunnel() {
        if (getLastKomut() != "tunel" && getLastKomut() != "Mission_Completed" && getLastKomut() != "ileri_sola") {
            appendKomut("tunel");
        }

        if (getLastKomut() == "tunel") {
            if (distance_ > 1) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                appendKomut("ileri_sola");
            }
        } 
        else if (getLastKomut() == "ileri_sola") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 10) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 10 && mesafe_ <= 11) {
                current_steering_angle_ = 110.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 11 && mesafe_ <= 21) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 21 && mesafe_ <= 22) {
                current_steering_angle_ = 110.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 22 && mesafe_ <= 28) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 28) {
                appendKomut("Mission_Completed");
                mesafe_ = 0;
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            commandBool_ = 0;
            appendKomut("no_command_tunel");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Tunel Gorevi Bitti!");
        }
    }
*/
    void forwardRightForced() {
        if (getLastKomut() != "ileri_saga_mecburi" && getLastKomut() != "Mission_Completed" && getLastKomut() != "ileri_saga") {
            appendKomut("ileri_saga_mecburi");
        }

        if (getLastKomut() == "ileri_saga_mecburi") {
            if (distance_ > 1) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                appendKomut("ileri_saga");
            }
        } 
        else if (getLastKomut() == "ileri_saga") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 12) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 12) {
                appendKomut("Mission_Completed");
                mesafe_ = 0;
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            commandBool_ = 0;
            appendKomut("no_command_forwardRightForced");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Ileri Saga Mecburi Gorevi Bitti!");
        }
    }

    void straight() {
        if (getLastKomut() != "Duz" && getLastKomut() != "Duz_Komutu" && getLastKomut() != "Mission_Completed") {
            appendKomut("Duz");
        }

        if (getLastKomut() == "Duz") {
            if (distance_ > 1) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                appendKomut("Duz_Komutu");
            }
        } 
        else if (getLastKomut() == "Duz_Komutu") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 8) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 8) {
                appendKomut("Mission_Completed");
                mesafe_ = 0;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            commandBool_ = 0;
            appendKomut("no_command_Duz");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Duz Gitme Gorevi Bitti!");
        }
    }
/*
    void intersection1() {
        if (getLastKomut() != "Kavsak_Basit" && getLastKomut() != "kavsak_duz_git" && 
            getLastKomut() != "kavsak_donus" && getLastKomut() != "Mission_Completed") {
            appendKomut("Kavsak_Basit");
        }

        if (getLastKomut() == "Kavsak_Basit") {
            if (distance_ > 1) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                appendKomut("kavsak_duz_git");
            }
        } 
        else if (getLastKomut() == "kavsak_duz_git") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 3) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 3) {
                mesafe_ = 0;
                appendKomut("kavsak_donus");
            }
        } 
        else if (getLastKomut() == "kavsak_donus") {
            mesafe_ += wheelData_;
            if (mesafe_ < 8) {
                current_steering_angle_ = 135.0f;
                commandBool_ = 1;
            } else if (mesafe_ >= 8) {
                appendKomut("Mission_Completed");
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            mesafe_ = 0;
            commandBool_ = 0;
            appendKomut("kavsak_no_command");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Basit Kavsak Gorevi Bitti!");
        }
    }
*/
    void intersection2() {
        if (getLastKomut() != "Kavsak" && getLastKomut() != "kavsak_sol_donus" && getLastKomut() != "kavsak_sola_don" && getLastKomut() != "missionCompleted") {
            appendKomut("Kavsak");
        }

        if (getLastKomut() == "Kavsak") {
            if (distance_ > 1) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                current_steering_angle_ = 100.0f;
                appendKomut("kavsak_sol_donus");
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "kavsak_sol_donus") {
            mesafe_ += wheelData_;
            if (mesafe_ <= 6) {
                current_steering_angle_ = 105.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 6 && mesafe_ <= 8) {
                appendKomut("kavsak_sola_don");
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "kavsak_sola_don") {
            mesafe_ += wheelData_;
            if (mesafe_ > 8 && mesafe_ <= 10) {
                current_steering_angle_ = 75.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 10 && mesafe_ <= 37) {
                current_steering_angle_ = 25.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 37 && mesafe_ <= 40) {
                // Şerit yoksa 75 (Burada sabitliyoruz, Python'daki lines kontrolü yerine)
                current_steering_angle_ = 75.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 40 && mesafe_ <= 43) {
                current_steering_angle_ = 110.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 43) {
                appendKomut("missionCompleted");
            }
        } 
        else if (getLastKomut() == "missionCompleted") {
            mesafe_ = 0;
            commandBool_ = 0;
            appendKomut("duraga_yanastim");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Kavsak Gorevi Bitti!");
        }
    }
    
    void notTurnLeft() {
        if (getLastKomut() != "sola_donme" && getLastKomut() != "sol_donme" && getLastKomut() != "Mission_Completed") {
            appendKomut("sola_donme");
        }

        if (getLastKomut() == "sola_donme") {
            if (distance_ > 1) { // Kamera Bounding box mesafesi (Yaklaşana kadar şerit takip et)
                commandBool_ = 1;
                current_steering_angle_ = detected_lane_angle_;
            } else {
                appendKomut("sol_donme");
                mesafe_ = 0;
            }
        } 
        else if (getLastKomut() == "sol_donme") {
            mesafe_ += wheelData_;

            if (mesafe_ <= 1) {
                current_steering_angle_ = 90.0f;
                commandBool_ = 1;
            } else if (mesafe_ > 1) {
                appendKomut("Mission_Completed");
                mesafe_ = 0;
                commandBool_ = 1;
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            commandBool_ = 0;
            mesafe_ = 0;
            appendKomut("no_command_notTurnLeft");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Sola Donulmez Gorevi Bitti!");
        }
    }

    void tasitGiremez() {
        if (getLastKomut() != "tasit_giremez" && getLastKomut() != "sola_donuyorum" && getLastKomut() != "Mission_Completed") {
            appendKomut("tasit_giremez");
        }

        if (getLastKomut() == "tasit_giremez") {
            if (distance_ > 5) {
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
            } else {
                appendKomut("sola_donuyorum");
            }
        } 
        else if (getLastKomut() == "sola_donuyorum") {
            mesafe_ += wheelData_;

            if (mesafe_ <= 8) {
                current_steering_angle_ = 25.0f; // Sola kır
                commandBool_ = 1;
            } else if (mesafe_ > 8) {
                // Python'da burada lines kontrolü vardı, şerit görünce tamamlıyordu. 
                // Biz encoder mesafesi dolunca tamamlıyoruz.
                appendKomut("Mission_Completed");
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            commandBool_ = 0;
            mesafe_ = 0;
            appendKomut("tasit_no_command");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Tasit Giremez Gorevi Bitti! Seride Donuluyor.");
        }
    }
    
    void stopManeuver() {
        if (getLastKomut() != "sukrudur" && getLastKomut() != "bitti") {
            appendKomut("sukrudur");
        }
        
        if (getLastKomut() == "sukrudur") {
            if (distance_ <= 1) { 
                RCLCPP_INFO(this->get_logger(), "Durdum, bekliyorum...");
                commandBool_ = 1;
                
                auto_mode_pub_->publish(std_msgs::msg::Bool().set__data(false));
                rclcpp::sleep_for(std::chrono::seconds(5)); 
                
                RCLCPP_INFO(this->get_logger(), "Harekete gectim");
                auto_mode_pub_->publish(std_msgs::msg::Bool().set__data(true));
                appendKomut("bitti");
            } else {
                commandBool_ = 1;
                current_steering_angle_ = detected_lane_angle_;
            }
        } 
        else if (getLastKomut() == "bitti") {
            RCLCPP_INFO(this->get_logger(), "Durma islemi bitti.");
            commandBool_ = 0;
            appendKomut("gorev_yok");
            active_task_ = "serit_takip"; 
        }
    }

    void turnRight() {
    if (getLastKomut() != "saga_don" && getLastKomut() != "Mission_Completed" && getLastKomut() != "sag") {
        appendKomut("saga_don");
    }

    if (getLastKomut() == "saga_don") {
        if (distance_ > 1) {
            current_steering_angle_ = detected_lane_angle_;
            commandBool_ = 1;
        } else {
            appendKomut("sag");
        }
    } 
    else if (getLastKomut() == "sag") {
        mesafe_ += wheelData_;
        
        if (mesafe_ <= 8) {
            commandBool_ = 1;
            current_steering_angle_ = 90.0f; // Önce biraz düz git
        } 
        else if (mesafe_ > 8 && mesafe_ <= 23) {
            current_steering_angle_ = 165.0f; // Tam sağa kır!
            commandBool_ = 1;
        } 
        else if (mesafe_ > 23) {
            // Python'daki efsanevi 85-100 derece Şeride Oturma Kontrolü!
            if (detected_lane_angle_ < 85.0f || detected_lane_angle_ > 100.0f) {
                current_steering_angle_ = 165.0f; // Araba düzleşene kadar sağa kırmaya devam!
                commandBool_ = 1;
            } else {
                // Araba şeride ip gibi oturdu, direksiyonu bırakabilirsin.
                current_steering_angle_ = detected_lane_angle_;
                commandBool_ = 1;
                appendKomut("Mission_Completed"); 
            }
        }
    } 
    else if (getLastKomut() == "Mission_Completed") {
        commandBool_ = 0;
        mesafe_ = 0;
        appendKomut("sagaDonusTamam");
        active_task_ = "serit_takip";
        RCLCPP_INFO(this->get_logger(), "Uzun Saga Donus Tamam!");
    }
}

    void turnLeft() {
        if (getLastKomut() != "sol" && getLastKomut() != "solun_yanindayim" && getLastKomut() != "Mission_Completed") {
            appendKomut("sol");
        }

        if (getLastKomut() == "sol") {
            if (distance_ > 35) {
                current_steering_angle_ = detected_lane_angle_; // Tabelaya yaklaşırken şeridi koru
                commandBool_ = 1;
            } else {
                appendKomut("solun_yanindayim");
            }
        }
        else if (getLastKomut() == "solun_yanindayim") {
            mesafe_ += wheelData_;
            
            if (mesafe_ <= 12) {
                current_steering_angle_ = 25.0f; // Sola kırmaya başla
                commandBool_ = 1;
            } else if (mesafe_ > 12) {
                // Şeride tam oturana ve araba düzleşene kadar (85-100 derece) sola dönüşe devam et!
                if (detected_lane_angle_ < 85.0f || detected_lane_angle_ > 100.0f) {
                    current_steering_angle_ = 25.0f; // Sola kırmaya devam!
                    commandBool_ = 1;
                } else {
                    // Araba düzlendi, direksiyonu şeride bırakabiliriz
                    current_steering_angle_ = detected_lane_angle_;
                    commandBool_ = 1;
                    appendKomut("Mission_Completed");
                }
            }
        } 
        else if (getLastKomut() == "Mission_Completed") {
            commandBool_ = 0;
            mesafe_ = 0;
            appendKomut("solaDonusTamam");
            active_task_ = "serit_takip";
            RCLCPP_INFO(this->get_logger(), "Uzun Sola Donus Tamam!");
        }
    }

    void station() {
    if (getLastKomut() != "durak" && getLastKomut() != "duraga_yanastim" && getLastKomut() != "durak_ici" && getLastKomut() != "serit_ara" && getLastKomut() != "Mission_Completed") {
        appendKomut("durak");
    }

    if (getLastKomut() == "durak") {
        if (distance_ > 25) {
            current_steering_angle_ = detected_lane_angle_; // Python'daki gibi şeridi koru
            commandBool_ = 1;
        } else {
            appendKomut("duraga_yanastim");
        }
    } 
    else if (getLastKomut() == "duraga_yanastim") {
        // Python kodunda: if labels[-1]=="duba": komut_tabela.append("Mission_Completed")
        // Eğer durakta bir engel/duba varsa görevi direkt bitir mantığı eklenmiş.
        if (last_seen_sign_ == "duba") {
            appendKomut("Mission_Completed");
            return; // Görevi hemen kes
        }

        mesafe_ += wheelData_;
        
        if (mesafe_ <= 1) {
            current_steering_angle_ = 90.0f;
            commandBool_ = 1;
        } else if (mesafe_ > 1 && mesafe_ <= 4) {
            current_steering_angle_ = detected_lane_angle_; // Python: steering_angle=steering_angle
            commandBool_ = 1;
        } else if (mesafe_ > 4 && mesafe_ <= 12) {
            current_steering_angle_ = 165.0f; // Tam sağa kır (Durağa gir)
            commandBool_ = 1;
        } else if (mesafe_ > 12 && mesafe_ <= 22) {
            current_steering_angle_ = 30.0f; // Sola toparla
            commandBool_ = 1;
        } else if (mesafe_ > 22) {
            current_steering_angle_ = 48.0f; // Arabayı düzle
            commandBool_ = 1;
            
            if (signalBool_ == 0) {
                RCLCPP_INFO(this->get_logger(), "Durakta uyuyorum...");
                auto_mode_pub_->publish(std_msgs::msg::Bool().set__data(false)); // Fren yap
                signalBool_ = 1;
                
                // Python'da time.sleep(5) olan kısım
                rclcpp::sleep_for(std::chrono::seconds(5)); // Yarışmada burayı 30 yapacaksın
                
                auto_mode_pub_->publish(std_msgs::msg::Bool().set__data(true)); // Tekrar gaza bas
                mesafe_ = 0;
                appendKomut("durak_ici");
            }
        }
    }
    else if (getLastKomut() == "durak_ici") {
        mesafe_ += wheelData_;
        
        if (mesafe_ <= 8) {
            current_steering_angle_ = 35.0f; // Duraktan çıkış açısı
            commandBool_ = 1;
        } else if (mesafe_ > 12) {
            current_steering_angle_ = detected_lane_angle_;
            commandBool_ = 1;
            appendKomut("serit_ara");
            mesafe_ = 0;
        }
    }
    else if (getLastKomut() == "serit_ara") {
        mesafe_ += wheelData_;
        
        if (mesafe_ >= 1 && mesafe_ <= 2) {
            current_steering_angle_ = 160.0f; // Son bir sağ toparlama
            commandBool_ = 1;
        } else if (mesafe_ > 2) {
            current_steering_angle_ = detected_lane_angle_;
            commandBool_ = 1;
            appendKomut("Mission_Completed");
        }
    }
    else if (getLastKomut() == "Mission_Completed") {
        commandBool_ = 0;
        mesafe_ = 0;
        signalBool_ = 0; // Çok Önemli! Bir sonraki durak görevi için fren kilidini açıyoruz.
        appendKomut("no_command_durak");
        active_task_ = "serit_takip";
        RCLCPP_INFO(this->get_logger(), "Durak Gorevi Basariyla Bitti!");
    }
}

    void readTrafficSignRed() {
    if (getLastKomut() != "kirmizi" && getLastKomut() != "kirmizi_bitti") {
        appendKomut("kirmizi");
    }

    if (getLastKomut() == "kirmizi") {
        if (distance_ > 25) {
            // Kırmızıya 25cm kalana kadar şeridi takip et
            current_steering_angle_ = detected_lane_angle_; 
            commandBool_ = 1;
        } else {
            // 25cm ve altındayız, fren yap ve şeritte kal!
            if (signalBool_ == 0) {
                auto_mode_pub_->publish(std_msgs::msg::Bool().set__data(false)); 
                signalBool_ = 1;
            }
            current_steering_angle_ = detected_lane_angle_;
            commandBool_ = 1;
            
            // Eğer kamera artık kırmızı ışığı GÖRMÜYORSA (Python'daki labels[-1] mantığı)
            // last_seen_sign_ değişkenini kullanarak kontrol ediyoruz.
            if (last_seen_sign_ != "kirmizi_isik") {
                appendKomut("kirmizi_bitti");
            }
        }
    } 
    else if (getLastKomut() == "kirmizi_bitti") {
        commandBool_ = 0;
        signalBool_ = 0;
        appendKomut("kirmizi_no_command");
        active_task_ = "serit_takip";
        RCLCPP_INFO(this->get_logger(), "Kirmizi Isik kayboldu, gorev bitti!");
    }
}

    // --- ANA ÇALIŞTIRICI (ROUTER) ---
    void controlLoop() {
        wheelData_ = 0; 

        if (task_handlers_.find(active_task_) != task_handlers_.end()) {
            task_handlers_[active_task_](); 
        } else {
            commandBool_ = 0;
        }

        std_msgs::msg::Float32 steer_msg;
        
        if (commandBool_ == 1) {
            steer_msg.data = current_steering_angle_; 
        } else {
            steer_msg.data = detected_lane_angle_; 
            
            std_msgs::msg::Bool auto_msg;
            auto_msg.data = true;
            auto_mode_pub_->publish(auto_msg);
        }
        
        steering_pub_->publish(steer_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OvatKomutNode>());
    rclcpp::shutdown();
    return 0;
}
