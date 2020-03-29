#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <boost/program_options.hpp>
#include "rotary_encoder.hpp"
#include "mqtt/async_client.h"

std::atomic_int rotary_encoder_ticks;

double velocity = .01;

class pwm {
    double pwm_freq;
    double pwm_range;
    int pin;
    bool verbose;
    
    public:
    pwm(int pin, int frequency=0, bool verbose=false){
        pwm_freq = gpioSetPWMfrequency(pin, frequency);
        pwm_range = gpioGetPWMrange(pin);
        this->pin = pin;
        this->verbose = verbose;
        
        if(verbose){
            std::cout << "\nSet PWM[" << pin << "] frequency to: " << pwm_freq;
            std::cout << "\nPWM[" << pin << "] range: " << pwm_range;
        }
    }
    int operator()(double dc){
        unsigned int udc = dc * pwm_range;
        gpioPWM(pin, udc);
        if(verbose)
            std::cout << "\nSet PWM[" << pin << "] dc to: " << udc;
        return udc;
    }
};

class elapsed_timer {
    std::chrono::time_point<std::chrono::steady_clock> start, stop; 
    
    public:
    elapsed_timer() {
        start = std::chrono::steady_clock::now();
    }
    
    double read() {
        stop = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = stop-start;
        double seconds = diff.count();
        return seconds;
    }
    
    double toc()
    {
        double rval = this->read();
        start = stop;
        return rval;
    }
};



void rotary_encoder_callback(int way)
{
    static elapsed_timer retimer{};
    double seconds = retimer.toc();
    
    if(seconds == 0)
        seconds = 1;
    double velstep = velocity * 1/seconds;
    if(velstep < 1.0)
        velstep = 1;
    int b = way * floor(velstep);
    rotary_encoder_ticks -= b;
//  std::cout << "\nvel=" << 1/seconds << ", way= " << way << ", step= " << b << ", pos=" << rotary_encoder_ticks;

}

int main(int ac, char **av)
{
    using namespace std;
    
    namespace po = boost::program_options;

    const int    QOS = 1;
    const auto PERIOD = std::chrono::seconds(5);
    const int MAX_BUFFERED_MSGS = 120;  // 120 * 5sec => 10min off-line buffering
    const std::string PERSIST_DIR { "data-persist" };

    
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("server",  po::value<std::string>()->default_value("localhost"), "mqtt server")
        ("port",    po::value<int>()->default_value(1883),"mqtt port")
        ("x",   po::value<int>()->default_value(7), "rotary encoder GPIOx")
        ("y",   po::value<int>()->default_value(8), "rotary encoder GPIOy")
        ("sw",  po::value<int>()->default_value(9), "pushbutton GPIO")
        ("pwm1",    po::value<int>()->default_value(20), "PWM1 output GPIO")
        ("pwm2",    po::value<int>()->default_value(21), "PWM2 output GPIO")
        ("pwm1topic",   po::value<std::string>()->default_value("pwm1"), "mqtt publish topic for pwm1 output")
        ("pwm2topic",   po::value<std::string>()->default_value("pwm2"), "mqtt publish topic for pwm2 output")
        ("topic1",  po::value<std::string>()->default_value("rotenc"), "mqtt publish topic for rotary encoder")
        ("topic2",  po::value<std::string>()->default_value("rotenc_btn"), "mqtt publish topic for rotary encoder pushbutton")
        ("velocity",    po::value<double>()->default_value(.01), "rotary encoder velocity accelerator")
        ("wdt", po::value<double>()->default_value(10), "pwm to zero after this many seconds of no PWM messages")
        ("verbose", "report extra data to stdout")
        ("csv", "csv readout to stdout")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);    

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }
    bool verbose = false;
    if (vm.count("verbose")){
        std::cout << "verbose set\n";
        verbose = true;
    }


    if(verbose){
        std::cout << "mqtt server set to " << vm["server"].as<std::string>() << "\n";
        std::cout << "mqtt port set to " << vm["port"].as<int>() << "\n";
        std::cout << "mqtt topic for rotary encoder is set to " << vm["topic1"].as<std::string>() << "\n";
        std::cout << "mqtt topic for rotary encoder pushbutton is set to " << vm["topic2"].as<std::string>() << "\n";
        std::cout << "rotary encoder velocity is set to " << vm["velocity"].as<double>() << "\n";
        std::cout << "pwm watchdog set to " << vm["wdt"].as<double>() << " seconds\n";
        std::cout << "rotary encoder GPIOx is set to " << vm["x"].as<int>() << "\n";
        std::cout << "rotary encoder GPIOy is set to " << vm["y"].as<int>() << "\n";
        std::cout << "PWM1 output GPIO is set to " << vm["pwm1"].as<int>() << "\n";
        std::cout << "PWM2 output GPIO is set to " << vm["pwm2"].as<int>() << "\n";
        std::cout << "mqtt subscribe topic for pwm1 output is set to " << vm["pwm1topic"].as<std::string>() << "\n";
        std::cout << "mqtt subscribe topic for pwm2 output is set to " << vm["pwm2topic"].as<std::string>() << "\n";
    }
    int gpiox, gpioy, gpiosw, port, gpiopwm1, gpiopwm2;
    std::string server, topic1, topic2, pwm1topic, pwm2topic;
    bool mode_csv = false;
    double wdt;
    
    gpiox =             vm["x"].        as<int>();
    gpioy =             vm["y"].        as<int>();
    gpiosw =            vm["sw"].       as<int>();
    port =              vm["port"].     as<int>();
    gpiopwm1 =          vm["pwm1"].     as<int>();
    gpiopwm2 =          vm["pwm2"].     as<int>();
    velocity =          vm["velocity"]. as<double>();
    server =            vm["server"].   as<std::string>();
    topic1 =            vm["topic1"].   as<std::string>();
    topic2 =            vm["topic2"].   as<std::string>();
    pwm1topic =         vm["pwm1topic"].as<std::string>();
    pwm2topic =         vm["pwm2topic"].as<std::string>();
    wdt =               vm["wdt"].      as<double>();
    
    if (vm.count("csv")){
        mode_csv = true;
        if(verbose)
            std::cout << "csv output mode is set\n";
    }

    std::cout << "done parsing command line\n" << std::flush;
    
    // mqtt
    std::string address = std::string("tcp://") + server + std::string(":") + std::to_string(port);
    mqtt::async_client cli(address, "", MAX_BUFFERED_MSGS, PERSIST_DIR);
    
    mqtt::topic pub_topic1(cli, topic1, QOS, true);
    mqtt::topic pub_topic2(cli, topic2, QOS, true);
    
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(MAX_BUFFERED_MSGS * PERIOD);
    connOpts.set_clean_session(true);
    connOpts.set_automatic_reconnect(true);

    std::cout << "Connecting to server '" << address << "'..." << std::flush;
    cli.connect(connOpts)->wait();
    std::cout << "OK\n" << std::endl;
    std::string payload;        
    
    // mqtt consume topics
    cli.start_consuming();
    std::cout << "Subscribing to topic[s]\n";
    cli.subscribe(pwm1topic, QOS)->wait();
    std::cout << "\t" << pwm1topic;
    cli.subscribe(pwm2topic, QOS)->wait();
    std::cout << "\n\t" << pwm2topic << "\n";
    mqtt::const_message_ptr mqtt_msg;
    
    // gpio
    if (gpioInitialise() < 0) return 1;

    re_decoder dec(gpiox, gpioy, rotary_encoder_callback);

    // wdt
    elapsed_timer wdt_pwm1{};
    elapsed_timer wdt_pwm2{};
    
    // pwm
    pwm pwmgen1{gpiopwm1};
    pwm pwmgen2{gpiopwm2};
    
    do{
        int b = rotary_encoder_ticks;
        rotary_encoder_ticks = 0;
        //std::atomic_exchange(&rotary_encoder_ticks,b);
        if(b != 0){
            pub_topic1.publish(std::to_string(b));
            std::cout << "\n" << topic1 << " " << b;
        }
        if(cli.try_consume_message(&mqtt_msg)){
            std::string topic = mqtt_msg->get_topic();
            std::string payload = mqtt_msg->to_string();
            if(topic == pwm1topic){
                double elapsed = wdt_pwm1.toc();
                std::cout << "t=" << elapsed << ", pwm1=" << payload << std::endl;
                pwmgen1(std::stod(payload));
            }
            if(topic == pwm2topic){
                double elapsed = wdt_pwm2.toc();
                std::cout << "t=" << elapsed << ", pwm2=" << payload << std::endl;
                pwmgen2(std::stod(payload));
            }
        }
        if(wdt_pwm1.read() > wdt){
            std::cout << "\nwdt_pwm1 timeout";
            pwmgen1(0);
            wdt_pwm1.toc();
        }
        if(wdt_pwm2.read() > wdt){
            std::cout << "\nwdt_pwm2 timeout";
            pwmgen2(0);
            wdt_pwm2.toc();
        }
        
        this_thread::sleep_for(chrono::milliseconds(100));
    }while(true);

    dec.re_cancel();

    gpioTerminate();
}

