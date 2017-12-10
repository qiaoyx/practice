def heapify(L, idx, size):
    l = idx * 2 + 1
    r = idx * 2 + 2

    largest = idx
    if l < size and L[l] < L[idx]:
        largest = l
    if r < size and L[r] < L[largest]:
        largest = r

    if largest is not idx:
        t = L[idx]
        L[idx] = L[largest]
        L[largest] = t
        heapify(L, largest, size)


def build_heap(L):
    for i in range(len(L) / 2 - 1, -1, -1):
        heapify(L, i, len(L))


def heap_sort(L):
    build_heap(L)
    for i in range(len(L) - 1, 0, -1):
        t = L[0]
        L[0] = L[i]
        L[i] = t
        heapify(L, 0, i)


def heap_insert(L, x):
    pass


def heap_maximun(L):
    pass


def extract_max(L):
    pass


def increase_key(L, idx, x):
    '''中文注释'''
    pass


def quick_sort(L, low, high):
    l = low
    #    l = high / 2
    h = high
    key = L[l]

    if (l >= h):
        return

    while l < h:
        while l < h and L[h] > key:
            h = h - 1
        L[l] = L[h]
        while l < h and L[l] < key:
            l = l + 1
        L[h] = L[l]
    L[l] = key

    quick_sort(L, low, l - 1)
    quick_sort(L, h + 1, high)


def count_sort(L, low, high):
    if low >= high:
        return

    count = []
    for i in range(high - low + 2):
        count.append(0)
    for item in L:
        if item > high:
            continue
        count[item - low] = count[item - low] + 1

    holder = 0
    for i in range(len(count)):
        for j in range(count[i]):
            L[holder] = i + low
            holder = holder + 1


L = [1, 6, 7, 9, 2, 3, 5, 4, 8, 10]
print '-->'
# print L
# heap_sort(L)
# print L

print 'quick_sort'
quick_sort(L, 0, len(L) - 1)
# count_sort(L, 1, 10)
print L


L = [1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5]
single = 0
for l in L:
    single = single ^ l
print single
