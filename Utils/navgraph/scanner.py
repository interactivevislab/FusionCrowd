class Scanner:
    def __init__(self, filename):
        self._filename = filename
        self._file = None
        self._line = None
        self._idx = 0

    def next_int(self):
        return int(next(self))

    def next_float(self):
        return float(next(self))

    def next_str(self):
        return str(next(self))

    def __iter__(self):
        return self

    def __next__(self):
        if self._file is None or len(self._line) == 0:
            raise StopIteration

        result = self._line[self._idx]
        self._idx += 1
        if self._idx == len(self._line):
            self._idx = 0
            self._line = self._file.readline().strip().split()

        return result

    def __enter__(self):
        self._file = open(self._filename, "r")
        self._line = self._file.readline().strip().split()
        self._idx = 0

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._file is not None:
            self._file.close()
            self._file = None

        return False
