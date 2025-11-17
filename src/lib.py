import numpy as np
import time
import threading

class deque:
    def __init__(self, size, n_channels=1, dtype=float):
        """
        size: number of samples to store
        n_channels: number of ADC channels (1 for single-channel)
        """
        self.size = size
        self.n_channels = n_channels
        self.buffer = np.zeros((size, n_channels), dtype=dtype)
        self.timestamps = np.zeros(size)
        self.head_index = 0  # Points to next write position
        self.tail_index = 0  # Points to oldest element
        self.count = 0       # Number of elements in buffer
        self.lock = threading.Lock()

    def append(self, value, timestamp):
        """
        value: single sample or array of shape (n_channels,)
        timestamp: optional, defaults to current time
        """
        if timestamp is None:
            timestamp = time.time()

        with self.lock:
            self.buffer[self.head_index] = value
            self.timestamps[self.head_index] = timestamp
            self.head_index = (self.head_index + 1) % self.size
            
            if self.count < self.size:
                self.count += 1
            else:
                self.tail_index = (self.tail_index + 1) % self.size

    def get(self):
        """Return data in correct time order (oldest → newest)."""
        with self.lock:
            if self.count == 0:
                return np.array([]), np.array([])
            
            indices = np.arange(self.tail_index, self.tail_index + self.count) % self.size
            return self.timestamps[indices].copy(), self.buffer[indices].copy()
    
    def clear(self):
        """Reset the deque to empty state."""
        with self.lock:
            self.head_index = 0
            self.tail_index = 0
            self.count = 0