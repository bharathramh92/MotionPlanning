class MinHeap:

    """This class is for Object and key for which the priority has to built should be passed while initializing. Build heap method will be called once the object is instantiated. updateData will also call the build heap method with the new data."""

    def __init__(self, data, key):
        """
        MinHeap data structure.

        :param data: iteratable data
        :param key: function to point the comparable variable.
        :return: None.
        """
        self.aHeapsize = 0
        self.data = data
        self.key = key
        self.build_heap()

    def updateData(self, data, key):
        """
        This method populates data and updates key if a heap object has to be reused.

        :param data: iteratable data
        :param key: function to point the comparable variable.
        :return: None
        """
        self.data = data
        self.key = key
        self.build_heap()

    def _parent(self, i):
        return (i-1)//2

    def _left(self, i):
        return 2*i + 1

    def _right(self, i):
        return 2*i+2

    def min_heapify(self, i):
        """
        Performs min heapify at index i.

        :param i: index
        :return: None
        """

        l = self._left(i)
        r = self._right(i)
        smallest = i

        if l < self.aHeapsize and self.key(self.data[l]) < self.key(self.data[i]):
            smallest = l

        if r < self.aHeapsize and self.key(self.data[r]) < self.key(self.data[smallest]):
            smallest = r

        if smallest != i:
            if smallest == l:
                if l < self.aHeapsize:
                    self.data[l].position = i
                self.data[i].position = l
                if r < self.aHeapsize:
                    self.data[r].position = r
            elif smallest == r:
                if r < self.aHeapsize:
                    self.data[r].position = i
                self.data[i].position = r
                if l < self.aHeapsize:
                    self.data[l].position = l

            self.data[smallest], self.data[i] = self.data[i], self.data[smallest]
            self.min_heapify(smallest)
        else:
            if l < self.aHeapsize:
                self.data[l].position = l
            self.data[i].position = i
            if r < self.aHeapsize:
                self.data[r].position = r

    def build_heap(self):                                                       # O(n)
        """
        Builds heap from self.data.

        :return: None
        """
        self.aHeapsize = len(self.data)
        loc_i = self._parent(self.aHeapsize - 1)
        while loc_i >= 0:
            self.min_heapify(loc_i)
            loc_i -= 1
        return self.data

    def extract_min(self):                                                       # O(log(n))
        """
        To extract min element from the heap.

        :return: Min element object
        """
        if self.aHeapsize < 1:
            return None

        minimum = self.data[0]
        self.data[0] = self.data[self.aHeapsize-1]
        self.aHeapsize -= 1
        if self.aHeapsize > 0:
            self.min_heapify(0)
        return minimum

    def decrease_key(self, node, newValue, setKeyFunction):
        """
        Decrease the key by bubble up operation.

        :param node: The object whose priority has to decreased
        :param newValue: The new value of the Key in the object
        :param setKeyFunction: Key setter function in object
        :return: Boolean
        """
        if newValue > self.key(node):          # new value should be smaller than the old one
            return False

        i = node.position                                                       # index
        setKeyFunction(newValue)
        while i > 0 and self.key(self.data[self._parent(i)]) > self.key(self.data[i]):
            self.data[self._parent(i)].position, self.data[i].position = \
                self.data[i].position, self.data[self._parent(i)].position
            self.data[self._parent(i)], self.data[i] = self.data[i], self.data[self._parent(i)]
            i = self._parent(i)
        return True

    def __len__(self):
        return self.aHeapsize
