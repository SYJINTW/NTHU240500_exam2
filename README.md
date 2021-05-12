<!-- ABOUT THE PROJECT -->
## About The Project

Exam 2

### Built With

* C++
* Python
* json

### Equipment List

* PC or notebook
* B_L4S5I_IOT01A
* uLCD display

<!-- GETTING STARTED -->
## Getting Started

### Running & compile

* Compile project.
  
    ```sh
    sudo mbed compile --source . --source ~/ee2405/mbed-os/ -m B_L4S5I_IOT01A -t GCC_ARM -f
    ```

* Input command from screen.
    
    ```sh
    sudo screen /dev/ttyACM0
    ```

* Inside Screen. Way to command.
    
    * Run SAFE MODE 
        
        ```sh
        /CAPTURE/run 0
        ```

    * Run DETECTION MODE 
        
        ```sh
        /CAPTURE/run 1
        ```


<!-- ROADMAP -->
## Roadmap
1. Connect WIFI and MQTT: run in a WIFI_MQTT_thread and have high priority to aviod wifi disconnect.
    In this part, no only contain WIFI and MQTT initalize, but also contain mqtt_thread for preparing sending message in anytime, and contain the button interrupt for leaving GUI MODE, and DETECTION_thread (it will discuss below).
    However, we also need to prepare for stop MQTT after using. 
    <strong>Caution:</strong> need to change the internet ip address.

1. GUI MODE: run in a GUI_thread and check MODE to know if executing GUI part and predicting gesture. If we press the button, then we will trigger a interrupt and run <strong>publish_message</strong>. In publish_message, we will send messages by MQTT

1. DETECTION MODE: run in a DETECTION_thread (inside WIFI_MQTT_thread) and check MODE to know if executing detection mode. 
    The main part of detection is 

<!-- Screenshot -->
## Results

<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [Electronics Tutorials](https://www.electronics-tutorials.ws/filter/filter_2.html)
* [Embedded System Lab](https://www.ee.nthu.edu.tw/ee240500/)
