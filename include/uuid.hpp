#pragma once
#include <random>
#include <sstream>
#include <string>
// refrence: https://stackoverflow.com/questions/24365331/how-can-i-generate-uuid-in-c-without-using-boost-library

class UUID {
public:
    static std::string generateV4() {
        std::stringstream ss;
        ss << std::hex;

        for (int i = 0; i < 8; ++i) {
            ss << dis(gen);
        }

        ss << "-";

        for (int i = 0; i < 4; ++i) {
            ss << dis(gen);
        }

        ss << "-4";

        for (int i = 0; i < 3; ++i) {
            ss << dis(gen);
        }

        ss << "-";

        ss << dis2(gen);

        for (int i = 0; i < 3; ++i) {
            ss << dis(gen);
        }

        ss << "-";

        for (int i = 0; i < 12; ++i) {
            ss << dis(gen);
        }

        return ss.str();
    }

private:
    inline static std::random_device rd{};
    inline static std::mt19937 gen{rd()};
    inline static std::uniform_int_distribution<> dis{0, 15};
    inline static std::uniform_int_distribution<> dis2{8, 11};
};