import numpy as np

class Pose:
    def __init__(self, R=np.eye(3), t=np.zeros([3, 1])):
        self.R = R.reshape([3, 3])
        self.t = t.reshape([3, 1])

    def __mul__(self, other):
        if isinstance(other, Pose):
            new_R = self.R @ other.R
            new_t = self.R @ other.t + self.t
            return Pose(new_R, new_t)
        elif isinstance(other, np.ndarray):
            return self.R @ other.reshape([3,1]) + self.t
        else:
            raise TypeError(f"Unsupported type for multiplication. Self: {type(self)}, Other: {type(other)}")

    def inv(self):
        R_inv = self.R.T
        t_inv = -R_inv @ self.t
        return Pose(R_inv, t_inv)

    def __repr__(self):
        return f'Pose(R=\n{self.R},\nt=\n{self.t})'

    def __str__(self):
        return f'Pose(R=\n{self.R},\nt=\n{self.t})'

    def __eq__(self, other):
        return np.array_equal(self.R, other.R) and np.array_equal(self.t, other.t)

    def __ne__(self, other):
        return not self.__eq__(other)