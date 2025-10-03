#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "realsense/realsense.h"

using namespace cv;

// small test program to test the realsense camera with the implemented custom class realsense
int main() {

    try 
    {
    // Create a realsense object
    Realsense realsense;

    // create a window to show the color frame
    

    // Initialize the realsense object
    RUT::TimePoint time0;
    Realsense::RealsenseConfig config;
    realsense.init(time0, config);


    while (true) {

        // Get the next rgb frame
        Mat color_frame = realsense.next_rgb_frame_blocking();

        // If color frame is empty, continue
        if (color_frame.empty()) {
            std::cout << "Empty frame. Terminate" << std::endl;
            break;
        }

        // Show the color frame
        imshow("Color Frame", color_frame);
        char c = (char)waitKey(25);  //Allowing 25 milliseconds frame processing time and initiating break condition//
        if (c == 27) {  //If 'Esc' is entered break the loop//
            break;
        }

    }

    return 0;
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}


