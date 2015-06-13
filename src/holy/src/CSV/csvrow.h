#ifndef CSVROW_H
#define CSVROW_H

#include <vector>
#include <string>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>

namespace CSV {


class Row
{
public:
    std::string const& operator[](std::size_t index) const;
    std::size_t size() const;

    void readNextRow(std::istream& str);

private:
    std::vector<std::string>    m_data;
};

}

#endif // CSVROW_H
