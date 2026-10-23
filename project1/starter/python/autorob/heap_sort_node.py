def heapify(values):
    values = values.copy()
    for i in range(len(values) // 2 - 1, -1, -1):
    ##Loop math 2-1 this calc last parrent for what ever size list while range (start, stop, step) sp first -1 says stop before -1 so include zero and last means go backwards by one
        shift_down(values, i)
    return values    
 
def heap_sort(numbers):
    copy = numbers.copy()
    sorted_list = []

    copy = heapify(copy)

    while len(copy) > 0:

        first = copy[0]
        sorted_list.append(first)
        # remove last element
        last = copy.pop()
        if len(copy) > 0:
            copy[0] = last
            shift_down(copy, 0)

    return sorted_list

def shift_down(values, index):
    left = 2 * index + 1
    right = 2 * index + 2

    while len(values) > 0:
        smallest_val = index
        if left < len(values) and values[left] < values[smallest_val]:
            smallest_val = left

        if right < len(values) and values[right] < values[smallest_val]:
            smallest_val = right

        if smallest_val != index:
            values[index], values[smallest_val] = values[smallest_val], values[index]
            index = smallest_val
            left = 2 * index + 1
            right = 2 * index + 2
        else:
            break

    