#ifndef CSVITERATOR_H
#define CSVITERATOR_H

#include "csvrow.h"

namespace CSV {

class Iterator
{   
public:
    typedef std::input_iterator_tag  iterator_category;
    typedef Row                      value_type;
    typedef std::size_t              difference_type;
    typedef Row*                     pointer;
    typedef Row&                     reference;

    Iterator(std::istream& str);
    Iterator();

    // Pre Increment
    Iterator& operator++();
    // Post increment
    Iterator operator++(int);
    Row const& operator*()   const;
    Row const* operator->()  const;

    bool operator==(Iterator const& rhs);
    bool operator!=(Iterator const& rhs);

private:
    std::istream* m_str;
    Row m_row;
};

}

#endif // CSVITERATOR_H
