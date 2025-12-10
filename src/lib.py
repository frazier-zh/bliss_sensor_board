"""
    Project: BLISS
    Application: BLISS Sensor Board User Interface
    File: src/lib.py
    Description: Utility classes.
    Author: Fang Zihang (Dr.)
    Email: zh.fang@nus.edu.sg
    Affiliation: National University of Singapore
"""

import numpy as np
import time
import threading
from typing import Tuple, Optional

class deque:
    def __init__(self, n_channels: int, dtype=float, size:int = 2048):
        """
        size: number of samples to store
        n_channels: number of ADC n_channels (1 for single-channel)
        """
        self.n_channels = n_channels
        self.buffer = np.zeros((size, n_channels), dtype=dtype)
        self.size = size
        self.timestamps = np.zeros((size), dtype=float)
        self.head_index = 0  # Points to next write position
        self.tail_index = 0  # Points to oldest element
        self.count = 0       # Number of elements in buffer
        self._lock = threading.RLock()

    def append(self, value, timestamp):
        """
        value: single sample or array of shape (n_channels,)
        timestamp: optional, defaults to current time
        """
        if timestamp is None:
            timestamp = time.time()

        with self._lock:
            self.buffer[self.head_index] = value
            self.timestamps[self.head_index] = timestamp
            self.head_index = (self.head_index + 1) % self.size
            
            if self.count < self.size:
                self.count += 1
            else:
                self.tail_index = (self.tail_index + 1) % self.size
        return True

    def get(self):
        """Return data in correct time order (oldest → newest)."""
        with self._lock:
            if self.count == 0:
                return np.empty((0,)), np.empty((0, self.n_channels))
            
            indices = np.arange(self.tail_index, self.tail_index + self.count) % self.size
            return self.timestamps[indices].copy(), self.buffer[indices].copy()
    
    def clear(self):
        """Reset the deque to empty state."""
        with self._lock:
            self.head_index = 0
            self.tail_index = 0
            self.count = 0

class storage:
    def __init__(self, n_channels: int, dtype=float, size: int=360000):
        self.n_channels = n_channels
        self.dtype = dtype
        self.size = size

        self.count = 0
        self._timestamp = np.empty((size), dtype=float)
        self._data = np.empty((size, n_channels), dtype=dtype)
        self._lock = threading.Lock()

    def append(self, value, timestamp):
        if self.count >= self.size:
            return False
        with self._lock:
            self._timestamp[self.count] = timestamp
            self._data[self.count, :] = value
            self.count = self.count + 1
        return True

    def _read_ranges_indexes(self, start_idx: int, end_idx: int) -> Tuple[np.ndarray, np.ndarray]:
        """Read ranges by integer indexes [start_idx, end_idx) return copies."""
        ts = self._timestamp[start_idx:end_idx]
        d = self._data[start_idx:end_idx, :]
        return ts, d

    def _binary_search_timestamp(self, target_ts: float) -> int:
        """Binary search for first index i where timestamps[i] >= target_ts.
        Returns an integer index in [0, len].
        """
        if target_ts <= self._timestamp[0]:
            return 0
        if target_ts > self._timestamp[self.count - 1]:
            return self.count - 1

        left = 0
        right = self.count - 1
        while left < right:
            mid = (left + right) // 2
            mid_ts = float(self._timestamp[mid])
            if mid_ts < target_ts:
                left = mid + 1
            else:
                right = mid
        return left
    
    def get(self, start: Optional[float] = None, end: Optional[float] = None, max_points: Optional[int] = None):
        """Retrieve data for a timeframe.
        """
        if self.count == 0:
            return np.empty((0,)), np.empty((0, self.n_channels))

        # start and/or end provided
        # if provided None, interpret as file start or file end
        if start is None:
            start_idx = 0
        else:
            start_idx = self._binary_search_timestamp(start)
        if end is None:
            end_idx = self.count - 1
        else:
            end_idx = self._binary_search_timestamp(end)

        length = max(0, end_idx - start_idx)
        if length == 0:
            return np.empty((0,)), np.empty((0, self.n_channels))

        # if max_points requested, sample uniformly
        if max_points is None:
            max_points = length
        if length <= max_points:
            # read contiguous region
            ts = self._timestamp[start_idx:end_idx]
            d = self._data[start_idx:end_idx, :]
            return ts, d
        else:
            # uniform downsample indices
            indices = np.linspace(start_idx, end_idx, num=max_points, endpoint=True, dtype=np.int64)
            ts = self._timestamp[indices]
            d = self._data[indices, :]
            return ts, d
        
    def at(self, time: Optional[float] = None):
        if self.count == 0:
            return 0, np.zeros(self.n_channels)
        if time is None:
            return self._timestamp[self.count - 1], self._data[self.count - 1, :]
        time_idx = self._binary_search_timestamp(time)
        return self._timestamp[time_idx], self._data[time_idx, :]

    def clear(self):
        with self._lock:
            self.count = 0
    