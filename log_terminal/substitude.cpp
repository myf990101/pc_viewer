#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

auto split_string(const std::string& str, const std::string& delimiter) -> std::vector<std::string> {
    std::vector<std::string> result;
    std::string::size_type   start = 0;
    std::string::size_type   end   = str.find(delimiter);

    while (end != std::string::npos) {
        result.push_back(str.substr(start, end - start));
        start = end + delimiter.length();
        end   = str.find(delimiter, start);
    }

    result.push_back(str.substr(start));

    return result;
}

struct TimedLine {
    int         t = 0;    // time in ms
    std::string name;
    std::string all_line;
};

auto LoadOne(std::ifstream& ifs, TimedLine* data) -> bool {
    std::string line;
    if (!std::getline(ifs, line)) {
        std::cout << "file eof" << std::endl;
        return false;
    }

    std::istringstream iss(line);
    iss >> data->t >> data->name;

    if (!iss) {
        std::cerr << "err parsing:\n" << line << std::endl;
        exit(1);
    }

    data->all_line = line;
    return true;
}

auto LoadAll(std::ifstream& ifs) -> std::vector<TimedLine> {
    std::vector<TimedLine> all;

    TimedLine data;
    while (LoadOne(ifs, &data)) { all.push_back(data); }

    return all;
}

auto cat_str(const std::vector<std::string>& all_string) -> std::string {
    std::string s;
    for (int i = 0; i < all_string.size(); ++i) {
        s += all_string[i];
        if (i != all_string.size() - 1) {
            s += " ";
        }
    }

    return s;
}

auto replaceAtPosition(const std::string& str1, const std::string& str2) -> std::string {
    size_t pos11 = str1.find(' ', str1.find(' ') + 1);
    size_t pos12 = str1.find(' ', pos11 + 1);
    if (pos11 == std::string::npos) {
        std::cerr << "cont find" << std::endl;
        return str1;
    }

    size_t      pos21       = str2.find(' ', str2.find(' ') + 1);
    size_t      pos22       = str2.find(' ', pos11 + 1);
    std::string replacement = str2.substr(pos21 + 1, pos22 - pos21);
    std::string result      = str1;
    result.replace(pos11 + 1, pos12 - pos11, replacement);
    return result;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "pram file_laser file_printf file_out laser_name" << std::endl;
        return 1;
    }

    std::string file_laser(argv[1]), file_printf(argv[2]), file_out(argv[3]);

    std::ifstream ifs_laser(file_laser), ifs_printf(file_printf);
    std::ofstream ofs_printf(file_out);

    assert(ifs_laser);
    assert(ifs_printf);

    std::vector<TimedLine>           laser_data = LoadAll(ifs_laser);
    std::map<std::string, TimedLine> laser_data_dict;
    for (const auto& v : laser_data) { laser_data_dict[std::to_string(v.t) + v.name] = v; }

    TimedLine printf_data;
    int       n_printf_laser  = 0;
    int       n_laser_we_have = laser_data.size();
    int       n_substitude    = 0;

    while (LoadOne(ifs_printf, &printf_data)) {
        std::string id = std::to_string(printf_data.t) + printf_data.name;
        if (laser_data_dict.count(id) == 0) {
            ofs_printf << printf_data.all_line << std::endl;
            continue;
        }

        ++n_printf_laser;
        auto p = laser_data_dict.find(id);
        if (p == laser_data_dict.end()) {
            ofs_printf << printf_data.all_line << std::endl;
        } else {    // found
            auto        all_line_new = p->second.all_line;
            auto        all_line_old = printf_data.all_line;
            std::string replace_id   = replaceAtPosition(all_line_new, all_line_old);

            ofs_printf << replace_id << std::endl;

            ++n_substitude;
            laser_data_dict.erase(p);
        }
    }

    std::cout << "we-have_laser:" << n_laser_we_have << " printf-laser:" << n_printf_laser
              << " substitude:" << n_substitude << std::endl;
    for (const auto& x : laser_data_dict) { std::cout << x.first << " "; }

    std::cout << std::endl;
}