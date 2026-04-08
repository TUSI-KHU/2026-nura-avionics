#pragma once
#define MAXSIZE 10000

template <class T>
class Queue
{
public:
    Queue()
    {
        _size = 0;
        _max_size = MAXSIZE;
        _front = _rear = 0;
        _array = new T *[_max_size];
    }

    inline bool IsEmpty()
    {
        return _size == 0;
    }

    inline bool IsFull()
    {
        return _size == _max_size;
    }

    int size()
    {
        return _size;
    }

    inline void clear()
    {
        while (!IsEmpty())
            pop();
    }

    void push(T info)
    {
        if ((_rear + 1) % _max_size == _front)
            return;
        _array[_rear] = new T(info);
        _rear = (_rear + 1) % _max_size;
        _size++;
    }

    void pop()
    {
        if (_rear == _front)
            return;
        delete _array[_front];
        _front = (_front + 1) % _max_size;
        _size--;
    }

    T &front()
    {
        if (_rear != _front)
        {
            return *(_array[_front]);
        }
    }

    Queue<T> &operator=(const Queue<T> &ref)
    {
        clear();
        int ref_front = ref._front;
        while (ref_front != ref._rear)
        {
            push(*(ref._array[ref_front]));
            ref_front = (ref_front + 1) % _max_size;
        }
    }

    ~Queue()
    {
        while (!IsEmpty())
            pop();
    }

private:
    T **_array;
    int _size;
    int _max_size;
    int _front;
    int _rear;
};
