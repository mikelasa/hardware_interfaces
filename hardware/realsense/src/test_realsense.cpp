#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "realsense/realsense.h"

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

using namespace cv;

/** small test program to test the realsense camera with the implemented custom class realsense 
 * configuration enables color and depth streams, and shows them in separate windows.
 * takes the last frame to save it into directory
*/
int main() {

    // Create a realsense object
    Realsense realsense;

    //change configs
    Realsense::RealsenseConfig config;
    config.width = 1280;
    config.height = 720;
    config.framerate = 30;
    config.enable_color = true;
    config.enable_depth = true;
    config.align_depth_to_color = false;

    // create a window to show the color frame
    namedWindow("Video Player", WINDOW_AUTOSIZE);

    // Initialize the realsense object
    RUT::TimePoint time0;
    realsense.init(time0, config);

    while (true) {

        // if condition for when rgb or depth are enabled
        if (config.enable_color && !config.enable_depth) {

            // Get the next rgb frame
            Mat rgb_data = realsense.next_rgb_frame_blocking();

            // If color frame is empty, continue
            if (rgb_data.empty()) {
                std::cout << "Empty frame. Terminate" << std::endl;
                break;
            }

            // Show the color frame
            imshow("Color Frame", rgb_data);
        }
        else if (config.enable_depth && !config.enable_color) {

            // Get the next depth frame
            Mat depth_frame = realsense.next_depth_frame_blocking();

            // If depth frame is empty, continue
            if (depth_frame.empty()) {
                std::cout << "Empty frame. Terminate" << std::endl;
                break;
            }

            // Show the depth frame
            imshow("Depth Frame", depth_frame);
        }
        else if(config.enable_color && config.enable_depth) {

            Realsense::FramePair pair = realsense.next_pair_frame_blocking();

            // If pair is invalid, continue
            if (!pair.valid) {
                std::cout << "Empty frame. Terminate" << std::endl;
                break;
            }

            // Show the color and depth frames
            imshow("Color Frame", pair.rgb);
            imshow("Depth Frame", pair.depth);
        }
        else {
            std::cerr << "Error: no stream enabled." << std::endl;
            return -1;
        }

        char c = (char)waitKey(25);  //Allowing 25 milliseconds frame processing time and initiating break condition//
        if (c == 27) {  //If 'Esc' is entered break the loop//
            break;
        }
    }


    return 0;

}


