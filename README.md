# Single Joint Control Board (SJC)

- 3 Encoder inputs
- 2 Isolated inputs
- 2 Endstop inputs
- 2 Digital outputs
- Servomotor interface with current sense and pwm output

![image](https://github.com/tgiacomellibr/sjc/assets/11637388/8380d131-5d1e-4ce9-90b8-8f08c3ddb93b)

## Communication library:  

1. Install packages and deps.:
    ```
    sudo apt update
    sudo apt install build-essential git g++ cmake manpages-dev
    sudo apt install libmodbus-dev
    ```

2. Acquire SJC packages:
    ``` 
    git clone https://github.com/tgiacomellibr/sjc.git
    ```

3. Create build directory:
    ```
    cd sjc/software
    mkdir -p build && cd build
    ```

4. Compile and install
    > Example: cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local
    ```
    cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/install 
    make
    make install
    ```

