#ifndef UNRAVELER_HPP
#define UNRAVELER_HPP

namespace lfc
{
    template <class T>
    class Unraveler {
        public:
        virtual bool isEndReached() = 0;
        virtual T nextValue() = 0;
        virtual void reset() = 0;

        virtual ~Unraveler() {}
    };
}

#endif