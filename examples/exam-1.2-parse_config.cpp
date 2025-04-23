#include "ConfigReader.h"
#include <iostream>

using namespace std;

int main(int argc, char *argv[]) {
    try {

        string usages =
                " Parse configuration data from file! \n"
                " Usages: \n"
                "    parse_config [your_config_file] \n"
                " Examples: \n"
                "    parse_config ../examples/spp.ini \n";

        if (argc <= 1) {
            cout << "ERROR: you must input config file !" << endl;
            cout << usages << endl;
            exit(-1);
        }

        ConfigReader configReader(argv[1]); // Replace with your config file path

        try {
            int GPS = configReader.getValueAsInt("GPS");
            std::string navFile = configReader.getValueAsString("navFile");
            bool BD2 = configReader.getValueAsBool("BD2");
            double noiseGPS = configReader.getValueAsDouble("noiseGPSCode");

            std::cout << "GPS: " << GPS << std::endl;
            std::cout << "Nav File: " << navFile << std::endl;
            std::cout << "BD2: " << BD2 << std::endl;
            std::cout << "Noise of GPS Code: " << noiseGPS << std::endl;

            double noiseGLO = configReader.getValueAsDouble("noiseGLO");
            cout << noiseGLO << endl;
        }
        catch (std::runtime_error &e) {
            cerr << e.what() << endl;
            exit(-1);
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}