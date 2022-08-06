# ble_dog

Kinematics
    
    --kinematics_tester.cpp and (dogKinematics.cpp + dogKinematics.h) do the same thing, feel free to use either for now.
        Both sets of programs need to have the eigen-3.4.0 directory at the same level.
    
    --Linking and Compiling cpp file with header file
        Compile with:    g++ -I eigen-3.4.0 -c dogKinematics.cpp <main-cpp-file-with-.cpp-extension>
        Link with:       g++ dogKinematics.o <main-cpp-file-with-.o-extension> -o <executable-file-name>
Adding more for testing
