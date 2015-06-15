#include "csvrow.h"

namespace CSV {

const std::string &Row::operator[](std::size_t index) const
{
    return m_data[index];
}

std::size_t Row::size() const
{
    return m_data.size();
}

void Row::readNextRow(std::istream &str)
{
    std::string line;
    std::getline(str,line);

    std::stringstream   lineStream(line);
    std::string         cell;

    m_data.clear();
    while(std::getline(lineStream,cell,','))
    {
        m_data.push_back(cell);
    }
}

}
