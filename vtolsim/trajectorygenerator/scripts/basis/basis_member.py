import numpy as np


class BasisMember:
    def __init__(self, degree):
        raise NotImplementedError

    def evalDerivative(self, s, k):
        """
        Evaluate the kth derivative of this basis function at s in [0,1]
        """
        raise NotImplementedError

    def evalInnerProduct(self, other, k):
        """
        Evaluate the inner product of the kth derivative of this basis
        function with the kth derivative of another basis function.
        """
        raise NotImplementedError

    @classmethod
    def generate_basis_vector(cls, num_bases):
        """
        Given a class inherited from BasisMember as well as the
        number of basis functions to use, create a vector of basis functions
        of degree 0 to num_bases-1
        """
        basis_vector = []
        for j in range(num_bases):
            basis_vector.append(cls(j))

        return basis_vector

    @staticmethod
    def eval_basis_vector(basis_vector, s, k):
        """
        Evaluate the kth derivative of a vector of BasisMembers at s
        """
        num_bases = len(basis_vector)
        basis_dk_at_s = np.zeros(num_bases)

        for j in range(num_bases):
            basis_dk_at_s[j] = \
                basis_vector[j].evalDerivative(s, k)

        return basis_dk_at_s
