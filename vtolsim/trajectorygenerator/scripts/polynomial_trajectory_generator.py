
import numpy as np
from scipy.optimize import minimize

from trajectory import Trajectory
from polynomial_trajectory_member import PolynomialTrajectoryMember
from basis.basis_member import BasisMember


class PolynomialTrajectoryGenerator:
    def __init__(
            self,
            num_segments,
            num_bases,
            basis_class,
            continuity_derivative,
            smoothness_derivative,
            knot_constraint_structure):
        """
        Create a PolynomialTrajectoryGenerator object which can be used to
        create trajectories assuming they follow the same structure.

        The structure includes all of the arguments to this function:
        num_segments          - the number of segments
        num_bases             - the number of basis members
        basis_class           - the basis type
        continuity_derivative - the maximum derivative at which to enforce
                            continuity. Knot value constraints for derivatives
                            higher than this will not be considered.
        smoothness_derivative - the derivative at which to optimize for
                            smoothness (ie snap = 4)
        knot_constraint_structure
            A list of dictionaries specifying what knot constraints are desired
            Each dictionary should include the following members:
                - deriv: The derivative number (0 = pos, 1 = vel, etc)
                - knot_list: A list specifying indices of knot points to
                            constrain. Knot point indices fall
                            in [0, num_segments]. Points omitted from the
                            list will not be constrained.
        """
        self.num_segments = num_segments
        self.num_bases = num_bases

        self.basis_class = basis_class

        self.continuity_derivative = continuity_derivative
        self.smoothness_derivative = smoothness_derivative

        self.knot_constraint_structure = knot_constraint_structure

    def generate_member(
            self,
            knot_constraint_values,
            spline_matrix,
            segment_times):
        """
        Compute coefficients for and construct a PolynomialTrajectoryMember.

        knot_constraint_values:
            A list of dictionaries specifying the values to be enforced
            at the knot points.
            Each dictionary should include the following members:
                - deriv: The derivative number (0 = pos, 1 = vel, etc)
                - value_list: A list specifying values at the knot points.
                            These values correspond to the knot points given
                            when specifying knot_constraint_structure.
        """
        a_bar = self._generate_constraint_vector_a_bar(knot_constraint_values)
        continuity_zero_vec = np.zeros(
            (self.num_segments - 1) * self.continuity_derivative)

        a_vec = np.concatenate([a_bar, continuity_zero_vec], axis=0)

        member_coefficients = spline_matrix @ a_vec
        member_basis = self.basis_class.generate_basis_vector(self.num_bases)

        new_member = PolynomialTrajectoryMember(
            member_coefficients,
            member_basis,
            segment_times)

        return new_member

    def generate_trajectory(
            self,
            multi_knot_constraint_values,
            total_time=None,
            segment_times=None,
            optimize_segment_times=False):
        """
        Generate a trajectory satisfying multi_knot_constraint_values
        with optimal segment time ratios.
        """

        if total_time is None:
            total_time = float(self.num_segments)

        if segment_times is None:
            segment_times = (total_time/self.num_segments) * np.ones(self.num_segments)

        if optimize_segment_times:
            total_time = np.sum(segment_times)
            scale = 1/total_time**(self.smoothness_derivative)

            def obj_fn(segment_times):
                _, obj = self.generate_member_set(
                    segment_times,
                    multi_knot_constraint_values
                )
                return np.sqrt(obj) * scale

            def con_fn(segment_times):
                return total_time - np.sum(segment_times)

            bounds = [(0.001, total_time) for i in range(len(segment_times))]
            cons = [{'type': 'eq', 'fun': con_fn}]
            ret = minimize(obj_fn, segment_times, constraints=cons, method='slsqp', bounds=bounds)

            print("ret: ", ret)

            member_list, J = self.generate_member_set(
                ret.x,
                multi_knot_constraint_values
            )
        else:
            member_list, J = self.generate_member_set(
                segment_times,
                multi_knot_constraint_values
            )

        print("Obj: ", J)

        new_trajectory = Trajectory(member_list)

        return new_trajectory

    def generate_member_set(self, segment_times, multi_knot_constraint_values):
        """
        Compute coefficients for and construct multiple
        PolynomialTrajectoryMember objects, each having the same structure.
        Compute the minimum-derivative cost function as well

        multi_knot_constraint_values:
            A list of lists following the format specified by
            knot_constraint_values in generate_member()
        """
        spline_matrix, W_full = self._compute_spline_matrix(segment_times)

        member_list = []
        P_matrix = []
        for kcv in multi_knot_constraint_values:
            new_member = self.generate_member(kcv, spline_matrix, segment_times)
            member_list.append(
                new_member
            )
            P_matrix.append(new_member.coefficients)

        P_matrix = np.column_stack(P_matrix)

        J = np.trace(P_matrix.T @ W_full @ P_matrix)

        return member_list, J

    def _compute_spline_matrix(self, segment_times):
        """
        The real meat of the algorithm.
        Finds a (num_bases*num_segments X
            num_knot_constraints + num_segments*continuity_derivative)
        matrix which can be used to find polynomial coefficients satisfying
        unknown (at this time) knot point constraints)
        """

        # compute derivative matrix
        D = self._generate_derivative_matrix_D(
            self.num_segments,
            self.num_bases,
            self.basis_class,
            self.continuity_derivative,
            self.knot_constraint_structure,
            segment_times
        )

        # number of constraints
        c = D.shape[0]

        # find SVD of derivative matrix
        (U, Sig, Vh) = np.linalg.svd(D)
        V1 = Vh[:c, :].T
        V2 = Vh[c:, :].T

        W_full = self._compute_W_full(
            self.num_bases,
            self.basis_class,
            self.smoothness_derivative,
            segment_times
        )

        # multiply to get spline matrix
        spline_matrix = \
            ((np.eye(self.num_bases * self.num_segments)
                - V2 @ np.linalg.inv(V2.T @ W_full @ V2) @ V2.T @ W_full)
                @ V1 @ np.diag(Sig**-1) @ U.T)

        return spline_matrix, W_full

    @staticmethod
    def _generate_constraint_vector_a_bar(knot_constraint_values):
        """
        Given a list of knot constraint values, create the corresponding vector
        """
        kv_sorted = sorted(knot_constraint_values, key=lambda k: k['deriv'])
        a_bar = []
        for kv_deriv_dict in kv_sorted:
            for a in kv_deriv_dict['value_list']:
                a_bar.append(a)

        return np.array(a_bar)

    @classmethod
    def _generate_derivative_matrix_D(
            cls,
            num_segments,
            num_bases,
            basis_class,
            continuity_derivative,
            knot_constraint_structure,
            segment_times):

        D = []

        # Use knot_constraint_structure to construct D_ka
        ks_sorted = sorted(knot_constraint_structure, key=lambda k: k['deriv'])
        for ks_deriv_dict in ks_sorted:
            if ks_deriv_dict['deriv'] > continuity_derivative:
                raise ValueError(
                    "knot structure derivative = {} > \
                        continuity derivative = {}"
                    .format())

            # can specify all
            knot_list = ks_deriv_dict['knot_list']
            if isinstance(knot_list, str):
                if knot_list.lower() == 'all':
                    knot_list = np.arange(num_segments+1)

            D_ka = cls._generate_knot_constraint_matrix_Dka(
                num_segments,
                num_bases,
                basis_class,
                ks_deriv_dict['deriv'],
                knot_list,
                segment_times
            )
            D.append(D_ka)

        # ensure derivative continuity
        for k_ell in range(continuity_derivative):
            Dc_k = cls._generate_continuity_derivative_matrix_Dc_k(
                num_segments,
                num_bases,
                basis_class,
                k_ell,
                segment_times
            )
            D.append(Dc_k)

        return np.concatenate(D, axis=0)

    @classmethod
    def _generate_knot_constraint_matrix_Dka(
            cls,
            num_segments,
            num_bases,
            basis_class,
            ka,
            knot_list,
            segment_times):

        Dka = []
        for knot_i in knot_list:
            # convert negative indices for indexing from end
            if knot_i < 0:
                knot_i = num_segments + knot_i

            if knot_i < 0 or knot_i > num_segments:
                raise ValueError(
                    "Invalid knot index knot_i = {}".format(knot_i))
            elif knot_i < num_segments:
                # fix the starting point
                s = 0.0
            else:
                # fix the endpoint of the last segment
                s = 1.0
                # last segment index is one less than last knot index
                knot_i -= 1

            Dka.append(
                cls._generate_Phi_i_vector(
                    num_segments,
                    num_bases,
                    basis_class,
                    knot_i,
                    s,
                    ka
                )
                * (1./segment_times[knot_i])**ka
            )

        return np.array(Dka)

    @classmethod
    def _generate_continuity_derivative_matrix_Dc_k(
            cls,
            num_segments,
            num_bases,
            basis_class,
            k_ell,
            segment_times):

        Dc_k = []
        for knot_i in range(num_segments - 1):
            Phi_left = (cls._generate_Phi_i_vector(
                            num_segments,
                            num_bases,
                            basis_class,
                            knot_i,
                            1.0,
                            k_ell)
                        * (1./segment_times[knot_i])**k_ell)

            Phi_right = (cls._generate_Phi_i_vector(
                            num_segments,
                            num_bases,
                            basis_class,
                            knot_i + 1,
                            0.0,
                            k_ell)
                        * (1./segment_times[knot_i+1])**k_ell)

            Dc_k.append(
                Phi_left - Phi_right
            )

        return np.array(Dc_k)

    @classmethod
    def _compute_W_full(
        cls,
        num_bases,
        basis_class,
        smoothness_derivative,
        segment_times):

        # find min-derivative gramian
        W_single = cls._compute_min_derivative_gramian_W(
            num_bases,
            basis_class,
            smoothness_derivative
        )

        # put into block diagonal matrix
        W_full = np.kron(
            np.diag((1./segment_times)**(2 * smoothness_derivative - 1)),
            W_single)

        return W_full


    @staticmethod
    def _compute_min_derivative_gramian_W(
            num_bases,
            basis_class,
            k):
        """
        Compute the (num_bases X num_bases) matrix of
        inner products of the kth derivative of
        basis functions
        """

        basis_vector = \
            basis_class.generate_basis_vector(num_bases)

        W = np.zeros((num_bases, num_bases))

        for j_row in range(num_bases):
            for j_col in range(num_bases):
                W[j_row, j_col] = \
                    basis_vector[j_row].evalInnerProduct(
                        basis_vector[j_col], k)

        return W

    @staticmethod
    def _generate_Phi_i_vector(
            num_segments,
            num_bases,
            basis_class,
            seg_idx,
            s,
            k):

        if seg_idx > num_segments - 1:
            raise ValueError("Invalid segment index seg_idx = {}, \
                             num_segments = {}".format(seg_idx, num_segments))

        mn = num_segments * num_bases
        Phi_i = np.zeros(mn)

        basis_vector = basis_class.generate_basis_vector(num_bases)
        bv_eval_s_k = BasisMember.eval_basis_vector(basis_vector, s, k)

        Phi_i[(seg_idx*num_bases):num_bases*(seg_idx + 1)] = bv_eval_s_k

        return Phi_i
