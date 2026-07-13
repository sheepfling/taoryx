"""Linear-algebra primitives for cataloged coordinate transformations."""

from __future__ import annotations

from taoryx.contracts import Basis3, Frame, FrameVector3, Vector3


def transform_vector(vector: FrameVector3, basis: Basis3, *, to_frame: Frame) -> FrameVector3:
    """Transform a vector between the two frames represented by ``basis``.

    This is the generic unit-vector transformation from TAOS equations 2-93
    through 2-96 (catalog ID ``TAOS-ALG-COORD-020``). The basis vectors express
    the child frame in parent-frame components. Forward transformation expands
    child components in the parent basis; inverse transformation projects the
    parent vector onto those unit vectors.
    """

    if to_frame not in (basis.parent_frame, basis.child_frame):
        raise ValueError(f"basis does not connect to target frame {to_frame}")
    if vector.frame is to_frame:
        return vector
    if vector.frame is not basis.parent_frame and vector.frame is not basis.child_frame:
        raise ValueError(f"basis does not connect to source frame {vector.frame}")
    if not basis.is_orthonormal():
        raise ValueError("coordinate transformation basis must be orthonormal")
    if vector.frame is basis.child_frame and to_frame is basis.parent_frame:
        result = basis.first.scaled(vector.vector.x) + basis.second.scaled(vector.vector.y) + basis.third.scaled(vector.vector.z)
    else:
        result = Vector3(
            vector.vector.dot(basis.first),
            vector.vector.dot(basis.second),
            vector.vector.dot(basis.third),
        )
    return FrameVector3(result, to_frame)
####
