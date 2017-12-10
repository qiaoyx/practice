#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <vector>

static void heapify(std::vector<int> &array, size_t idx, size_t len)
{
    size_t left =  (idx << 1) + 1;
    size_t right = (idx << 1) + 2;

    std::cout << idx << ",";

    size_t largest = idx;
    if (left < len && array[left] > array[idx])
        largest = left;
    if (right < len && array[right] > array[largest])
        largest = right;

    if (largest != idx) {
        int tmp = array[idx];
        array[idx] = array[largest];
        array[largest] = tmp;

        heapify(array, largest, len);
    } else {
        std::cout << "ok" << std::endl;
    }
}

static void build_heap(std::vector<int> &array)
{
    for (int i = array.size() / 2 - 1; i >= 0; --i) {
        heapify(array, i, array.size());
    }
}

void heap_sort(std::vector<int> &array)
{
    build_heap(array);
    for (int i = array.size() - 1; i > 0; --i) {
        int tmp = array[0];
        array[0] = array[i];
        array[i] = tmp;
        heapify(array, 0, i);
    }
}

int main(int argc, char *argv[])
{
    std::vector<int> array;
    for (int i = 0; i < 10; ++i)
        array.push_back(i);

    heap_sort(array);
    std::cout << std::endl;

    for (size_t i = 0; i < array.size(); ++i)
        std::cout << array[i] << " ";
    std::cout << std::endl;
    return 0;
}
