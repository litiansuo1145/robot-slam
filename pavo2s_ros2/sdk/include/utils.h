#ifndef __UTILS_H__
#define __UTILS_H__

#include <queue>
#include <deque>
#include <algorithm>
#include <string>
#include <cstring>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "pavo2s_types.h"
#include "pavo2s_common.h"


const double PI = std::acos(-1);

#define Degree2Radians(x) ((x) * PI / 180.0)
const int NUM_ROT_ANGLES = 36001;

namespace pavo2s
{

template<typename T>
class SynchronizedQueue
{
public:
    SynchronizedQueue (std::size_t max_size = 0) :
        max_size_(max_size),
        queue_(), mutex_(), cond_(),
        request_to_end_(false), enqueue_data_(true)
    {
    }

    void enqueue (const T& data)
    {
        std::unique_lock<std::mutex> lock (mutex_);

        if (enqueue_data_)
        {
            if((max_size_ != 0) && queue_.size() >= max_size_) //exceed size limit
            {
                queue_.pop();
            }
            queue_.push (data);
            cond_.notify_one ();
        }
    }

    void enqueue(T&& data)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        if (enqueue_data_)
        {
            //queue_.push(std::move(data));
            queue_.push(std::forward<T>(data));
            cond_.notify_one();
        }
    }

    template <typename... _Args>
    void emplace(_Args&&... __args)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (enqueue_data_)
        {
            queue_.emplace(std::forward<_Args>(__args)...);
            cond_.notify_one();
        }
    }

    bool dequeue(T& result, uint16_t time_out = 0)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        if (time_out == 0)
        {
            while (queue_.empty())
            {
                cond_.wait(lock);
            }
        }
        else
        {
            while (queue_.empty())
            {
                if (cond_.wait_for(lock, std::chrono::milliseconds(time_out)) == std::cv_status::timeout)
                {
                    return false;
                }
            }
        }

        if (request_to_end_)
        {
            doEndActions();
            return false;
        }

        result = std::move(queue_.front());
        queue_.pop();

        return true;
    }

    bool tryDequeue (T& result)
    {
        std::unique_lock<std::mutex> lock (mutex_);

        if(queue_.empty ())
            return false;

        if (request_to_end_)
        {
            doEndActions ();
            return false;
        }

        result = std::move(queue_.front ());
        queue_.pop ();

        return true;
    }

    void stopQueue ()
    {
        std::unique_lock<std::mutex> lock (mutex_);
        request_to_end_ = true;
        cond_.notify_one ();
    }

    unsigned int size ()
    {
        std::unique_lock<std::mutex> lock (mutex_);
        return static_cast<unsigned int> (queue_.size ());
    }

    bool isEmpty () const
    {
        std::unique_lock<std::mutex> lock (mutex_);
        return (queue_.empty ());
    }

    void Clear()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while(!queue_.empty())
        {
            queue_.pop();
        }
    }
private:
    void doEndActions ()
    {
        enqueue_data_ = false;

        while (!queue_.empty ())
        {
            queue_.pop ();
        }
    }

    std::size_t max_size_;
    std::queue<T> queue_;            // Use STL queue to store data
    mutable std::mutex mutex_;       // The mutex to synchronise on
    std::condition_variable cond_;   // The condition to wait for
    bool request_to_end_;
    bool enqueue_data_;
};


class SinuLookupTable
{
public:
    static SinuLookupTable* Instance()
    {
        if(instance_== NULL)
        {
            instance_ = new SinuLookupTable();
        }
        return instance_;
    }

    double CosValue(int angle) //1/100 degree
    {
        int angle_mod = angle % (NUM_ROT_ANGLES - 1);
        return cos_lookup_table_[angle_mod];
    }

    double SinValue(int angle) //1/100 degree
    {
        int angle_mod = angle % (NUM_ROT_ANGLES - 1);
        return sin_lookup_table_[angle_mod];
    }

private:
    SinuLookupTable(int size = NUM_ROT_ANGLES)
    {
        InitTables(size);
    }

    void InitTables(int size)
    {
        if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
        {
            cos_lookup_table_.resize(size);
            sin_lookup_table_.resize(size);
            for (int i = 0; i < size; i++)
            {
                double rad = Degree2Radians(i / 100.0);
                cos_lookup_table_[i] = std::cos(rad);
                sin_lookup_table_[i] = std::sin(rad);
            }
        }
    }
private:
    static SinuLookupTable* instance_;
    std::vector<double> cos_lookup_table_;
    std::vector<double> sin_lookup_table_;

};

class NetCoverter
{

public:
    static bool String2Bytes(const std::string& ip_str, char* ip_bytes)
    {
        if(ip_str.empty())  //empty string
        {
            return false;
        }

        if (count(ip_str.begin(), ip_str.end(), '.') != 3)
        {
            return false;
        }

        std::string ip_str_copy(ip_str);
        TrimSpace(ip_str_copy); //remove leading and tail space

        std::string str_tmp;
        std::size_t prev_pos = 0;

        int dot_num = 0;
        int byte_single = 0;
        char tmp_bytes[4];

        for(std::size_t i=0; i<ip_str_copy.length(); i++)
        {
            if(ip_str_copy[i]=='.')
            {
                if(dot_num++ > 3)
                {
                    return false;
                }

                str_tmp = ip_str_copy.substr(prev_pos, i- prev_pos);
                prev_pos = i+1;

                byte_single = atoi(str_tmp.c_str());  //invalid value
                if(byte_single > 255)
                    return false;
                tmp_bytes[dot_num-1] = static_cast<char>(byte_single);

                continue;
            }
            if( (ip_str_copy[i]<'0') || (ip_str_copy[i] > '9') ) //invalid character
            {
                return false;
            }
        }

        str_tmp = ip_str_copy.substr(prev_pos);
        byte_single = atoi(str_tmp.c_str());
        tmp_bytes[dot_num] = static_cast<char>(byte_single);

        memcpy(ip_bytes, tmp_bytes, 4);

        return true;
    }

    static bool Bytes2String(std::string& ip_str, const char* ip_bytes)
    {
        if(ip_bytes == 0)
        {
            return false;
        }

        ip_str = std::to_string(static_cast<unsigned char>(ip_bytes[0])) + "."
                 + std::to_string(static_cast<unsigned char>(ip_bytes[1])) + "."
                 + std::to_string(static_cast<unsigned char>(ip_bytes[2])) + "."
                 + std::to_string(static_cast<unsigned char>(ip_bytes[3]));
        return true;
    }

    static bool Port2Bytes(const uint16_t& port_int, char* port_bytes)  //from uint16_t to char[4]
    {
        if(port_bytes == nullptr)
        {
            return false;
        }

        memset(port_bytes, 0, 4);
        port_bytes[2] = (port_int & 0xFF00) >> 8;
        port_bytes[3] = (port_int & 0x00FF);

        return true;
    }

    static bool Bytes2Port(uint16_t& port_int, const char* port_bytes)
    {
        if(port_bytes == nullptr)
        {
            return false;
        }

        port_int = (static_cast<unsigned char>(port_bytes[2]) << 8)
                   + static_cast<unsigned char>( port_bytes[3]);

        return true;
    }


private:
    static std::string& TrimSpace(std::string &str)
    {
        if (str.empty())
        {
            return str;
        }
        str.erase(0, str.find_first_not_of(" "));  //trim white space
        str.erase(str.find_last_not_of(" ") + 1);

        str.erase(0, str.find_first_not_of("\t"));  //trim tab
        str.erase(str.find_last_not_of("\t") + 1);

        return str;
    }
};
}

#endif //__UTILS_H__


