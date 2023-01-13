import numpy as np

from .basis_member import BasisMember


class StandardBasisMember(BasisMember):
    """
    Implementation of the standard polynomial basis
    {1 s s^2/2 s^3/3! ... s^n-1/(n-1)!}
    """
    def __init__(self, degree):
        self.degree = degree

    def evalDerivative(self, s, k):
        """
        Evaluate the kth derivative of this basis function at s in [0,1]
        """
        if s > 1.0 or s < 0.0:
            raise ValueError("Invalid basis value s = {}".format(s))

        if k > self.degree:
            return 0.0
        else:
            return (s**(self.degree - k))/np.math.factorial(self.degree - k)

    def evalInnerProduct(self, other, k):
        """
        Evaluate the inner product of the kth derivative of this basis
        function with the kth derivative of another basis function.
        """
        if k > self.degree or k > other.degree:
            return 0.0
        else:
            dself_deg = self.degree - k
            dother_deg = other.degree - k
            return 1.0 / \
                (np.math.factorial(dself_deg)
                    * np.math.factorial(dother_deg)
                    * (dself_deg + dother_deg + 1))
