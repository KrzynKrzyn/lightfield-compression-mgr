#ifndef RUN_LENGTH_ENCODER_HPP
#define RUN_LENGTH_ENCODER_HPP

#include <vector>

namespace lfc
{
    class RunLengthEncoder2
    {
    public:
        Unraveler<short> *unraveler;

    public:
        RunLengthEncoder2(Unraveler<short> *unraveler) : unraveler(unraveler) {}

        std::vector<short> encode(size_t minRunLength)
        {
            std::vector<short> encodedBytes;

            short byteRun = unraveler->nextValue();
            size_t runStart = 0;

            int i = 1;
            while (!unraveler->isEndReached())
            {
                short currentByte = unraveler->nextValue();

                if (currentByte == byteRun)
                {
                    // ++runStart;
                }
                else
                {
                    size_t runLength = i - runStart;
                    if (runLength >= minRunLength)
                    {
                        encodedBytes.push_back(byteRun);
                        encodedBytes.push_back(runLength + 4000);
                        // encodedBytes.push_back(byteRun);
                        // records.push_back(RunRecord(runStart, runLength));
                    }
                    else
                    {
                        for (int j = 0; j < runLength; ++j)
                            encodedBytes.push_back(byteRun);
                    }

                    byteRun = currentByte;
                    runStart = i;
                }

                ++i;
            }

            // TODO repetition
            size_t runLength = i - runStart;
            if (runLength >= minRunLength)
            {
                encodedBytes.push_back(byteRun);
                encodedBytes.push_back(runLength + 4000);
                // encodedBytes.push_back(byteRun);
                // records.push_back(RunRecord(runStart, runLength));
            }
            else
            {
                for (int j = 0; j < runLength; ++j)
                    encodedBytes.push_back(byteRun);
            }

            return encodedBytes;
        }
    };

    template <class T>
    class RunLengthEncoder
    {
    public:
        struct RunRecord
        {
            size_t runStart;
            size_t runLength;

            RunRecord(size_t runStart, size_t runLength) : runStart(runStart), runLength(runLength) {}
        };

        Unraveler<T> *unraveler;
        std::vector<RunRecord> records;

    public:
        RunLengthEncoder(Unraveler<T> *unraveler) : unraveler(unraveler) {}

        std::vector<T> encode(size_t minRunLength)
        {
            records.clear();
            std::vector<T> encodedBytes;

            // unsigned char byteRun = bytes[0];
            T byteRun = unraveler->nextValue();
            size_t runStart = 0;

            int i = 1;
            // for (size_t i = 1; i < bytes.size(); ++i)
            while (!unraveler->isEndReached())
            {
                T currentByte = unraveler->nextValue();
                // unsigned char currentByte = bytes[i];

                if (currentByte == byteRun)
                {
                    // ++runStart;
                }
                else
                {
                    size_t runLength = i - runStart;
                    if (runLength >= minRunLength)
                    {
                        encodedBytes.push_back(byteRun);
                        records.push_back(RunRecord(runStart, runLength));
                    }
                    else
                    {
                        for (int j = 0; j < runLength; ++j)
                            encodedBytes.push_back(byteRun);
                    }

                    byteRun = currentByte;
                    runStart = i;
                }

                // if (i % 500000 == 0)
                //     std::cout << "And push! " << i << std::endl;
                ++i;
            }

            // TODO repetition
            size_t runLength = i - runStart;
            if (runLength > minRunLength)
            {
                encodedBytes.push_back(byteRun);
                records.push_back(RunRecord(runStart, runLength));
            }
            else
            {
                for (int j = 0; j < runLength; ++j)
                    encodedBytes.push_back(byteRun);
            }

            return encodedBytes;
        }
    };
}

#endif