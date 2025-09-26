'use client';

import React, { useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import { useRobot } from '../contexts/RobotContext';

interface RobotArm3DProps {
  width?: number;
  height?: number;
  className?: string;
}

export const RobotArm3D: React.FC<RobotArm3DProps> = ({ 
  width = 800, 
  height = 600, 
  className = '' 
}) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const sceneRef = useRef<THREE.Scene | undefined>(undefined);
  const rendererRef = useRef<THREE.WebGLRenderer | undefined>(undefined);
  const cameraRef = useRef<THREE.PerspectiveCamera | undefined>(undefined);
  const armGroupRef = useRef<THREE.Group | undefined>(undefined);
  const targetArmGroupRef = useRef<THREE.Group | undefined>(undefined);
  const animationIdRef = useRef<number | undefined>(undefined);
  const [isInitialized, setIsInitialized] = useState(false);
  const [isClientMounted, setIsClientMounted] = useState(false);

  const { armState, robotConfig, targetAngles } = useRobot();
  
  // Ensure we only initialize on client side after hydration
  useEffect(() => {
    setIsClientMounted(true);
  }, []);
  

  // Mouse controls
  const mouseRef = useRef({
    isDown: false,
    prevX: 0,
    prevY: 0,
    rotationX: 0,
    rotationY: 0
  });

  useEffect(() => {
    if (!containerRef.current || !canvasRef.current || !robotConfig || !isClientMounted) return;
    
    // Get actual container dimensions
    const rect = containerRef.current.getBoundingClientRect();
    const actualWidth = rect.width;
    const actualHeight = rect.height;
    
    // Don't proceed if container has no dimensions
    if (actualWidth === 0 || actualHeight === 0) {
      return;
    }
    

    // Scene setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x404040);
    sceneRef.current = scene;

    // Camera setup
    const camera = new THREE.PerspectiveCamera(75, actualWidth / actualHeight, 0.1, 1000);
    camera.position.set(50, 50, 50);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    // Renderer setup bound to our React canvas
    const renderer = new THREE.WebGLRenderer({ canvas: canvasRef.current, antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    renderer.setSize(actualWidth, actualHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.setClearColor(0x404040);
    rendererRef.current = renderer;

    // Add grid (slightly darker regular lines, same center line)
    const gridHelper = new THREE.GridHelper(100, 20, 0x888888, 0x666666);
    scene.add(gridHelper);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(50, 50, 50);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048;
    scene.add(directionalLight);

    // Create arm groups (current and target overlays)
    const armGroup = new THREE.Group();
    scene.add(armGroup);
    armGroupRef.current = armGroup;

    const targetArmGroup = new THREE.Group();
    // Render target overlay slightly on top
    targetArmGroup.renderOrder = 1;
    scene.add(targetArmGroup);
    targetArmGroupRef.current = targetArmGroup;

    // Fix global 90° axis mismatch (convert Z-up to Y-up)
    armGroup.rotation.x = -Math.PI / 2;
    targetArmGroup.rotation.x = -Math.PI / 2;

    // No manual appendChild — React owns the canvas

    // Mouse event handlers for orbit controls
    const handleMouseDown = (event: MouseEvent) => {
      mouseRef.current.isDown = true;
      mouseRef.current.prevX = event.clientX;
      mouseRef.current.prevY = event.clientY;
    };

    const handleMouseMove = (event: MouseEvent) => {
      if (!mouseRef.current.isDown) return;

      const deltaX = event.clientX - mouseRef.current.prevX;
      const deltaY = event.clientY - mouseRef.current.prevY;

      mouseRef.current.rotationY += deltaX * 0.01;
      mouseRef.current.rotationX += deltaY * 0.01;

      // Clamp vertical rotation
      mouseRef.current.rotationX = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, mouseRef.current.rotationX));

      mouseRef.current.prevX = event.clientX;
      mouseRef.current.prevY = event.clientY;
    };

    const handleMouseUp = () => {
      mouseRef.current.isDown = false;
    };

    const handleWheel = (event: WheelEvent) => {
      event.preventDefault();
      const camera = cameraRef.current;
      if (!camera) return;

      const scale = event.deltaY > 0 ? 1.1 : 0.9;
      camera.position.multiplyScalar(scale);
      
      // Clamp distance
      const distance = camera.position.length();
      if (distance < 10) {
        camera.position.normalize().multiplyScalar(10);
      } else if (distance > 200) {
        camera.position.normalize().multiplyScalar(200);
      }
    };

    // Add event listeners
    const canvasEl = canvasRef.current;
    canvasEl.addEventListener('mousedown', handleMouseDown);
    canvasEl.addEventListener('mousemove', handleMouseMove);
    canvasEl.addEventListener('mouseup', handleMouseUp);
    canvasEl.addEventListener('wheel', handleWheel, { passive: false });

    // Animation loop
    let frameCount = 0;
    const animate = () => {
      frameCount++;
      if (frameCount === 1) {
        console.log('RobotArm3D: Animation loop started');
      }
      
      const camera = cameraRef.current;
      if (camera) {
        // Update camera position based on mouse rotation
        const radius = camera.position.length();
        camera.position.x = radius * Math.sin(mouseRef.current.rotationY) * Math.cos(mouseRef.current.rotationX);
        camera.position.y = radius * Math.sin(mouseRef.current.rotationX);
        camera.position.z = radius * Math.cos(mouseRef.current.rotationY) * Math.cos(mouseRef.current.rotationX);
        camera.lookAt(0, 0, 0);
        
        renderer.render(scene, camera);
      }
      animationIdRef.current = requestAnimationFrame(animate);
    };

    animate();
    setIsInitialized(true);

    // Cleanup
    return () => {
      if (animationIdRef.current) {
        cancelAnimationFrame(animationIdRef.current);
      }
      
      const canvasElCleanup = canvasRef.current;
      if (canvasElCleanup) {
        canvasElCleanup.removeEventListener('mousedown', handleMouseDown);
        canvasElCleanup.removeEventListener('mousemove', handleMouseMove);
        canvasElCleanup.removeEventListener('mouseup', handleMouseUp);
        canvasElCleanup.removeEventListener('wheel', handleWheel as any);
      }
      
      // Do not remove the canvas; React owns it
      renderer.dispose();
      setIsInitialized(false);
    };
  }, [robotConfig, isClientMounted]);

  // Handle resize
  useEffect(() => {
    if (!isInitialized || !containerRef.current) return;

    const handleResize = () => {
      const renderer = rendererRef.current;
      const camera = cameraRef.current;
      
      if (renderer && camera && containerRef.current) {
        const rect = containerRef.current.getBoundingClientRect();
        const newWidth = rect.width;
        const newHeight = rect.height;
        
        
        // Update renderer size
        renderer.setSize(newWidth, newHeight);
        
        // Update camera aspect ratio
        camera.aspect = newWidth / newHeight;
        camera.updateProjectionMatrix();
      }
    };

    // Listen to window resize
    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
    };
  }, [isInitialized]);

  // Build robot arm model from DH parameters (current state)
  useEffect(() => {
    if (!robotConfig || !armGroupRef.current || !isInitialized) return;

    // Clear previous arm model
    while (armGroupRef.current.children.length > 0) {
      armGroupRef.current.remove(armGroupRef.current.children[0]);
    }

    const armGroup = armGroupRef.current;
    let currentTransform = new THREE.Matrix4();

    robotConfig.joints.forEach((joint, index) => {
      // Current joint angle (from armState or default to config theta)
      const currentAngle = armState.joints[index]?.currentAngle ?? joint.theta;
      const theta = THREE.MathUtils.degToRad(currentAngle);
      const alpha = THREE.MathUtils.degToRad(joint.alpha);
      const d = joint.d;
      const a = joint.a;

      // Create joint cylinder (representing the joint/motor)
      const jointGeometry = new THREE.CylinderGeometry(2, 2, 3, 16);
      const jointMaterial = new THREE.MeshLambertMaterial({ 
        color: index === 0 ? 0xff6b6b : 0x4ecdc4 // Different color for base
      });
      const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);
      jointMesh.castShadow = true;
      jointMesh.receiveShadow = true;

      // Create link cylinder (representing the arm segment)
      if (a > 0 || d > 0) {
        const linkLength = Math.sqrt(a * a + d * d);
        const linkGeometry = new THREE.CylinderGeometry(1, 1, linkLength, 8);
        const linkMaterial = new THREE.MeshLambertMaterial({ 
          color: 0x888888,
          transparent: true,
          opacity: 0.8
        });
        const linkMesh = new THREE.Mesh(linkGeometry, linkMaterial);
        linkMesh.castShadow = true;
        linkMesh.receiveShadow = true;

        // Orient cylinder from local origin toward (a, 0, d)
        if (linkLength > 0) {
          const dir = new THREE.Vector3(a, 0, d);
          const mid = dir.clone().multiplyScalar(0.5);
          // Cylinder default axis is +Y; rotate so +Y aligns to dir
          const quat = new THREE.Quaternion().setFromUnitVectors(
            new THREE.Vector3(0, 1, 0),
            dir.clone().normalize()
          );
          linkMesh.position.copy(mid);
          linkMesh.setRotationFromQuaternion(quat);
        }

        // Group joint and link
        const segmentGroup = new THREE.Group();
        segmentGroup.add(jointMesh);
        segmentGroup.add(linkMesh);

        // Apply DH transformation
        const transform = new THREE.Matrix4();
        
        // DH transformation matrix
        // T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
        const cosTheta = Math.cos(theta);
        const sinTheta = Math.sin(theta);
        const cosAlpha = Math.cos(alpha);
        const sinAlpha = Math.sin(alpha);

        transform.set(
          cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta,
          sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta,
          0, sinAlpha, cosAlpha, d,
          0, 0, 0, 1
        );

        // Combine with previous transformations
        currentTransform.multiply(transform);

        // Apply transformation to segment
        segmentGroup.matrix.copy(currentTransform);
        segmentGroup.matrixAutoUpdate = false;

        armGroup.add(segmentGroup);
      } else {
        // Just add the joint if no link
        const segmentGroup = new THREE.Group();
        segmentGroup.add(jointMesh);

        // Apply DH transformation
        const transform = new THREE.Matrix4();
        const cosTheta = Math.cos(theta);
        const sinTheta = Math.sin(theta);
        const cosAlpha = Math.cos(alpha);
        const sinAlpha = Math.sin(alpha);

        transform.set(
          cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta,
          sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta,
          0, sinAlpha, cosAlpha, d,
          0, 0, 0, 1
        );

        currentTransform.multiply(transform);
        segmentGroup.matrix.copy(currentTransform);
        segmentGroup.matrixAutoUpdate = false;

        armGroup.add(segmentGroup);
      }
    });

    // Add end effector
    const endEffectorGeometry = new THREE.SphereGeometry(1.5, 16, 16);
    const endEffectorMaterial = new THREE.MeshLambertMaterial({ color: 0xff4757 });
    const endEffector = new THREE.Mesh(endEffectorGeometry, endEffectorMaterial);
    endEffector.castShadow = true;
    
    const endEffectorGroup = new THREE.Group();
    endEffectorGroup.add(endEffector);
    endEffectorGroup.matrix.copy(currentTransform);
    endEffectorGroup.matrixAutoUpdate = false;
    
    armGroup.add(endEffectorGroup);

  }, [robotConfig, armState, isInitialized]);

  // Build target overlay arm model from DH parameters (semi-transparent, no color)
  useEffect(() => {
    if (!robotConfig || !targetArmGroupRef.current || !isInitialized) return;

    // Clear previous target arm model
    while (targetArmGroupRef.current.children.length > 0) {
      targetArmGroupRef.current.remove(targetArmGroupRef.current.children[0]);
    }

    const armGroup = targetArmGroupRef.current;
    let currentTransform = new THREE.Matrix4();

    robotConfig.joints.forEach((joint, index) => {
      // Target joint angle from context or fallback to current targetAngles array
      const targetAngleDeg = targetAngles[index] ?? joint.theta;
      const theta = THREE.MathUtils.degToRad(targetAngleDeg);
      const alpha = THREE.MathUtils.degToRad(joint.alpha);
      const d = joint.d;
      const a = joint.a;

      // Joint (wireframe/transparent)
      const jointGeometry = new THREE.CylinderGeometry(2, 2, 3, 16);
      const jointMaterial = new THREE.MeshBasicMaterial({ color: 0xdddddd, opacity: 0.3, transparent: true, wireframe: true });
      const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);

      // Link
      if (a > 0 || d > 0) {
        const linkLength = Math.sqrt(a * a + d * d);
        const linkGeometry = new THREE.CylinderGeometry(1, 1, linkLength, 8);
        const linkMaterial = new THREE.MeshBasicMaterial({ color: 0xdddddd, opacity: 0.22, transparent: true });
        const linkMesh = new THREE.Mesh(linkGeometry, linkMaterial);

        // Orient cylinder from local origin toward (a, 0, d)
        if (linkLength > 0) {
          const dir = new THREE.Vector3(a, 0, d);
          const mid = dir.clone().multiplyScalar(0.5);
          const quat = new THREE.Quaternion().setFromUnitVectors(
            new THREE.Vector3(0, 1, 0),
            dir.clone().normalize()
          );
          linkMesh.position.copy(mid);
          linkMesh.setRotationFromQuaternion(quat);
        }

        const segmentGroup = new THREE.Group();
        segmentGroup.add(jointMesh);
        segmentGroup.add(linkMesh);

        const transform = new THREE.Matrix4();
        const cosTheta = Math.cos(theta);
        const sinTheta = Math.sin(theta);
        const cosAlpha = Math.cos(alpha);
        const sinAlpha = Math.sin(alpha);

        transform.set(
          cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta,
          sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta,
          0,        sinAlpha,             cosAlpha,            d,
          0, 0, 0, 1
        );

        currentTransform.multiply(transform);
        segmentGroup.matrix.copy(currentTransform);
        segmentGroup.matrixAutoUpdate = false;

        armGroup.add(segmentGroup);
      } else {
        const segmentGroup = new THREE.Group();
        segmentGroup.add(jointMesh);

        const transform = new THREE.Matrix4();
        const cosTheta = Math.cos(theta);
        const sinTheta = Math.sin(theta);
        const cosAlpha = Math.cos(alpha);
        const sinAlpha = Math.sin(alpha);

        transform.set(
          cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta,
          sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta,
          0,        sinAlpha,             cosAlpha,            d,
          0, 0, 0, 1
        );

        currentTransform.multiply(transform);
        segmentGroup.matrix.copy(currentTransform);
        segmentGroup.matrixAutoUpdate = false;

        armGroup.add(segmentGroup);
      }
    });

    // End effector overlay
    const endEffectorGeometry = new THREE.SphereGeometry(1.5, 16, 16);
    const endEffectorMaterial = new THREE.MeshBasicMaterial({ color: 0xdddddd, opacity: 0.28, transparent: true, wireframe: false });
    const endEffector = new THREE.Mesh(endEffectorGeometry, endEffectorMaterial);
    const endEffectorGroup = new THREE.Group();
    endEffectorGroup.add(endEffector);
    endEffectorGroup.matrix.copy(currentTransform);
    endEffectorGroup.matrixAutoUpdate = false;
    armGroup.add(endEffectorGroup);

  }, [robotConfig, targetAngles, isInitialized]);

  if (!robotConfig || !isClientMounted) {
    return (
      <div 
        className={`flex items-center justify-center bg-gray-900 text-gray-400 ${className}`}
      >
        <div className="text-center">
          <div className="text-4xl mb-4">🔧</div>
          <p className="text-lg">
            {!isClientMounted ? 'Initializing...' : 'Loading robot configuration...'}
          </p>
        </div>
      </div>
    );
  }

  return (
    <div ref={containerRef} className={`overflow-hidden relative ${className}`}>
      <canvas ref={canvasRef} className="absolute inset-0 w-full h-full block" />
    </div>
  );
};

export default RobotArm3D;
