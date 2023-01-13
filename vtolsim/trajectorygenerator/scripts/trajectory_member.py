#!/usr/bin/env python3
# TrajectoryMember base class - Defines the interface for a trajectory member


class TrajectoryMember:
    """
    A single member of a trajectory.
    For example if a trajectory consisted of x and y members,
    each would be represented by a trajectory member.
    """
    def eval(self, tau, k):
        """
        Evaulate the kth derivative at the time tau.
        """
        raise NotImplementedError
