#include "Simulation.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

int main(int argc, char** argv)
{
    if (argc < 7) {
        std::cerr << "Error: Please provide one map file, five task files, and a single output file.\n";
        return 1;  // Exit the program with an error code
    }

    // Map file is the first argument
    std::string mapFile = argv[1];

    // Task files and output file
    std::vector<std::string> taskFiles = { argv[2], argv[3], argv[4], argv[5], argv[6] };
    std::string outputFile = argv[7];

    // Open the output file once for writing all timing results
    std::ofstream outFile(outputFile);
    if (!outFile) {
        std::cerr << "Error: Unable to open output file.\n";
        return 1;
    }

    // Loop over each task file in the batch
    for (size_t i = 0; i < taskFiles.size(); ++i) {
        outFile << "=== Timing for task file " << (i + 1) << ": " << taskFiles[i] << " ===\n";
        std::cout << "=== Timing for task file " << (i + 1) << ": " << taskFiles[i] << " ===\n";

        // Initialize the Simulation object with the map file and current task file
        Simulation simu1(mapFile, taskFiles[i]);

        // Time profiling run_TOTP
        auto start = std::chrono::high_resolution_clock::now();
        simu1.run_TOTP();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        outFile << "Time taken by run_TOTP: " << elapsed.count() << " seconds\n";
        std::cout << "Time taken by run_TOTP: " << elapsed.count() << " seconds\n";

        // Profiling SavePath for simu1
        start = std::chrono::high_resolution_clock::now();
        simu1.SavePath("temp_tp_path.txt");  // Temporary file for timing
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        outFile << "Time taken by SavePath (simu1): " << elapsed.count() << " seconds\n";
        std::cout << "Time taken by SavePath (simu1): " << elapsed.count() << " seconds\n";

        // Create another simulation instance for TPTR
        Simulation simu2(mapFile, taskFiles[i]);

        // Time profiling run_TPTR
        start = std::chrono::high_resolution_clock::now();
        simu2.run_TPTR();
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        outFile << "Time taken by run_TPTR: " << elapsed.count() << " seconds\n";
        std::cout << "Time taken by run_TPTR: " << elapsed.count() << " seconds\n";

        // Profiling SavePath for simu2
        start = std::chrono::high_resolution_clock::now();
        simu2.SavePath("temp_tptr_path.txt");  // Temporary file for timing
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        outFile << "Time taken by SavePath (simu2): " << elapsed.count() << " seconds\n";
        std::cout << "Time taken by SavePath (simu2): " << elapsed.count() << " seconds\n";

        // Profiling ShowTask for simu2
        start = std::chrono::high_resolution_clock::now();
        simu2.ShowTask();
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        outFile << "Time taken by ShowTask (simu2): " << elapsed.count() << " seconds\n";
        std::cout << "Time taken by ShowTask (simu2): " << elapsed.count() << " seconds\n";

        outFile << "=== Completed timing for task file " << (i + 1) << " ===\n\n";
        std::cout << "=== Completed timing for task file " << (i + 1) << " ===\n\n";
    }

    // Close the output file
    outFile.close();
    std::cout << "Batch processing completed. Timing results saved to " << outputFile << "\n";

    // Clean up temporary files
    std::remove("temp_tp_path.txt");
    std::remove("temp_tptr_path.txt");

    return 0;
}
