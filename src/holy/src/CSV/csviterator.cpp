#include "csviterator.h"

namespace CSV {

std::istream& operator>>(std::istream& str,Row& data)
{
    data.readNextRow(str);
    return str;
}

Iterator::Iterator(std::istream &str) :m_str(str.good()?&str:NULL)
{ 
    ++(*this);
}

Iterator::Iterator() :m_str(NULL)
{
    
}

Iterator &Iterator::operator++()
{
    if (m_str)
    {
        (*m_str) >> m_row;
        m_str = m_str->good()?m_str:NULL;
    }
    return *this;
}

Iterator Iterator::operator++(int)
{
    Iterator tmp(*this);
    ++(*this);
    return tmp;
}

const Row &Iterator::operator*() const
{
    return m_row;
}

const Row *Iterator::operator->() const
{
    return &m_row;
}

bool Iterator::operator==(const Iterator &rhs)
{
    return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));
}

bool Iterator::operator!=(const Iterator &rhs)
{
    return !((*this) == rhs);
}

}
