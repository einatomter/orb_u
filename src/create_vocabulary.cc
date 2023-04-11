#include <iostream>
#include "ORBVocabulary.h"

int main(int argc, char *argv[]) {

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <vocabulary_file>" << std::endl;
        return 1;
    }

    string vocabulary_file = argv[1];

    std::cout << "running ORB SLAM convertToBinary.cpp" << std::endl;
    ORB_SLAM3::ORBVocabulary voc(vocabulary_file);
    voc.saveToBinFile("aqualoc.txt.bin");
    std::cout << "done" << std::endl;

    return 0;
}