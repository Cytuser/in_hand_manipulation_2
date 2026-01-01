"""
TCI (Task Compatibility Index) Helper Module

This module provides utilities for loading Pinocchio models and resolving
frame IDs for manipulability/TCI cost computation in DDP.

Following the Plugin Architecture:
- All Pinocchio instances are created independently
- No modification to ActionModel or QuasistaticSimulator required
"""

import os
import numpy as np
import pinocchio as pin
from typing import List, Tuple, Optional, Dict

from common.get_model_path import MODEL_PATH, SDF_PATH


class TCIHelper:
    """
    Helper class for TCI/Manipulability cost computation.
    
    Provides static methods for:
    1. Loading standalone Pinocchio models for hands
    2. Resolving fingertip frame IDs
    3. Computing sphere center from state
    """
    
    # Default mesh directories for Allegro Hand
    ALLEGRO_MESH_DIRS = [
        os.path.join(MODEL_PATH, "migrate_from_drake"),
    ]
    
    # Default mesh directories for LEAP Hand
    LEAP_MESH_DIRS = [
        os.path.join(MODEL_PATH, "migrate_from_drake"),
    ]
    
    # Allegro Hand fingertip link names (index, middle, ring, thumb)
    ALLEGRO_TIP_LINKS = ["link_3", "link_7", "link_11", "link_15"]
    
    # LEAP Hand fingertip link names
    LEAP_TIP_LINKS = ["fingertip", "fingertip_2", "fingertip_3", "thumb_fingertip"]
    
    @staticmethod
    def load_hand_model_from_sdf(
        sdf_path: str,
        mesh_dirs: Optional[List[str]] = None
    ) -> Tuple[pin.Model, pin.Data]:
        """
        Load a standalone Pinocchio model from SDF file.
        
        This creates an INDEPENDENT Pinocchio instance that does not
        interfere with any existing dynamics computation.
        
        Args:
            sdf_path: Path to the SDF file
            mesh_dirs: List of directories containing mesh files
            
        Returns:
            Tuple of (pin.Model, pin.Data)
        """
        if mesh_dirs is None:
            mesh_dirs = TCIHelper.ALLEGRO_MESH_DIRS
            
        # Build model from SDF - creates independent instance
        pin_model, _, _, _ = pin.buildModelsFromSdf(
            filename=sdf_path,
            package_dirs=mesh_dirs
        )
        
        # Create data object for this model
        pin_data = pin_model.createData()
        
        return pin_model, pin_data
    
    @staticmethod
    def load_allegro_hand_model(
        sdf_filename: str = "allegro_3d_4finger.sdf",
        mesh_dirs: Optional[List[str]] = None
    ) -> Tuple[pin.Model, pin.Data]:
        """
        Load Allegro Hand Pinocchio model.
        
        Args:
            sdf_filename: Name of the SDF file in the sdf directory
            mesh_dirs: Optional custom mesh directories
            
        Returns:
            Tuple of (pin.Model, pin.Data)
        """
        sdf_path = os.path.join(SDF_PATH, sdf_filename)
        
        if mesh_dirs is None:
            mesh_dirs = TCIHelper.ALLEGRO_MESH_DIRS
            
        return TCIHelper.load_hand_model_from_sdf(sdf_path, mesh_dirs)
    
    @staticmethod
    def load_leap_hand_model(
        sdf_filename: str = "leap_3d_4finger_tac3d_simple.sdf",
        mesh_dirs: Optional[List[str]] = None
    ) -> Tuple[pin.Model, pin.Data]:
        """
        Load LEAP Hand Pinocchio model.
        
        Args:
            sdf_filename: Name of the SDF file
            mesh_dirs: Optional custom mesh directories
            
        Returns:
            Tuple of (pin.Model, pin.Data)
        """
        sdf_path = os.path.join(SDF_PATH, sdf_filename)
        
        if mesh_dirs is None:
            mesh_dirs = TCIHelper.LEAP_MESH_DIRS
            
        return TCIHelper.load_hand_model_from_sdf(sdf_path, mesh_dirs)
    
    @staticmethod
    def get_frame_ids_by_names(
        pin_model: pin.Model,
        frame_names: List[str]
    ) -> List[int]:
        """
        Get frame IDs from frame names.
        
        Args:
            pin_model: Pinocchio model
            frame_names: List of frame/link names
            
        Returns:
            List of frame IDs
        """
        frame_ids = []
        for name in frame_names:
            try:
                fid = pin_model.getFrameId(name)
                frame_ids.append(fid)
            except Exception as e:
                raise ValueError(f"Frame '{name}' not found in model. "
                               f"Available frames: {[f.name for f in pin_model.frames]}")
        return frame_ids
    
    @staticmethod
    def get_allegro_fingertip_frame_ids(pin_model: pin.Model) -> List[int]:
        """
        Get Allegro Hand fingertip frame IDs.
        
        Args:
            pin_model: Pinocchio model of Allegro Hand
            
        Returns:
            List of 4 fingertip frame IDs [index, middle, ring, thumb]
        """
        return TCIHelper.get_frame_ids_by_names(
            pin_model, 
            TCIHelper.ALLEGRO_TIP_LINKS
        )
    
    @staticmethod
    def get_leap_fingertip_frame_ids(pin_model: pin.Model) -> List[int]:
        """
        Get LEAP Hand fingertip frame IDs.
        
        Args:
            pin_model: Pinocchio model of LEAP Hand
            
        Returns:
            List of 4 fingertip frame IDs
        """
        return TCIHelper.get_frame_ids_by_names(
            pin_model,
            TCIHelper.LEAP_TIP_LINKS
        )
    
    @staticmethod
    def get_joint_id_by_name(
        pin_model: pin.Model,
        joint_name: str
    ) -> int:
        """
        Get joint ID from joint name.
        
        Args:
            pin_model: Pinocchio model
            joint_name: Name of the joint
            
        Returns:
            Joint ID
        """
        if pin_model.existJointName(joint_name):
            return pin_model.getJointId(joint_name)
        else:
            raise ValueError(f"Joint '{joint_name}' not found in model. "
                           f"Available joints: {list(pin_model.names)}")
    
    @staticmethod
    def extract_hand_config_from_state(
        x: np.ndarray,
        hand_joint_indices: List[int]
    ) -> np.ndarray:
        """
        Extract hand joint configuration from full state vector.
        
        Args:
            x: Full state vector [object_state, hand_joints, ...]
            hand_joint_indices: Indices of hand joints in state
            
        Returns:
            Hand joint configuration
        """
        return x[hand_joint_indices]
    
    @staticmethod
    def compute_sphere_center_fixed(
        center_position: np.ndarray
    ) -> np.ndarray:
        """
        Return fixed sphere center position.
        
        For fixed-center rotation tasks, the sphere center is constant.
        
        Args:
            center_position: 3D position of sphere center
            
        Returns:
            Sphere center position (3,)
        """
        return np.asarray(center_position).flatten()[:3]
    
    @staticmethod
    def print_model_info(pin_model: pin.Model) -> None:
        """
        Print Pinocchio model information for debugging.
        
        Args:
            pin_model: Pinocchio model
        """
        print("=" * 50)
        print("Pinocchio Model Information")
        print("=" * 50)
        print(f"Number of joints: {pin_model.njoints}")
        print(f"Number of frames: {len(pin_model.frames)}")
        print(f"Configuration dimension (nq): {pin_model.nq}")
        print(f"Velocity dimension (nv): {pin_model.nv}")
        print("\nJoints:")
        for i, name in enumerate(pin_model.names):
            print(f"  [{i}] {name}")
        print("\nFrames:")
        for frame in pin_model.frames:
            print(f"  [{frame.name}] parent_joint={frame.parent}")
        print("=" * 50)

