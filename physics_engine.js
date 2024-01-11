/**
 * File: physics_engine.js
 * Project: 3D Rigid Body Physics Engine - Collisions and Motion
 * Description: Contains the logic for the Rigid Body 3D physics engine including:
 * Collision Detection and Response, Physics Calculations, Movement Logic, Robot Setup, and Engine Setup
 * Author: Alberto Chavez Garcia
 * Created: June 23, 2023
 * Last Modified: August 16, 2023
 */
 

/**
 * Initializes the ThreeJS scene.
 * Performs the camera and floor setup.
 * 
 * @returns {Object} components - An object containing:
 * - {THREE.Scene} scene - The created scene
 * - {THREE.WebGLRenderer} renderer - The created WebGL renderer
 * - {THREE.OrthographicCamera} camera - The created orthographic camera
 */
function setup_scene() {
    
    const components = {};

    // Scene setup
    components.scene = new THREE.Scene();

    // Camera setup
    components.camera = new THREE.OrthographicCamera(
    window.innerWidth / -50, 
    window.innerWidth / 50,  
    window.innerHeight / 50,
    window.innerHeight / -50, 
    0.1, 
    100 
    );

    components.camera.position.set(20, 20, 20); 
    const target = new THREE.Vector3(0, 0, 0);
    components.camera.lookAt(target);    
    components.camera.position.z = 15;
    components.camera.position.x -= 5;
    components.camera.position.y -= 4;
    
    // Floor attributes and setup
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 512;
    const context = canvas.getContext('2d');
    const tile_size = 32;
    const num_tiles_x = canvas.width / tile_size;
    const num_tiles_y = canvas.height / tile_size;
    
    for (let x = 0; x < num_tiles_x; x++) {
        for (let y = 0; y < num_tiles_y; y++) {
            const color = (x + y) % 2 === 0 ? '#CCCCCC' : '#FFFFFF';
            context.fillStyle = color;
            context.fillRect(x * tile_size, y * tile_size, tile_size, tile_size);
        }
    }
    
    const texture = new THREE.CanvasTexture(canvas);
    const floor_geometry = new THREE.PlaneGeometry(40, 40);
    const floor_material = new THREE.MeshStandardMaterial({
        map: texture,
        roughness: 0.8,
        metalness: 0.2
    });

    const floor = new THREE.Mesh(floor_geometry, floor_material);
    floor.rotation.x = -Math.PI / 2;

    // Adding light to scene
    const ambient_light = new THREE.AmbientLight(0xffffff, 0.9);
    components.scene.add(ambient_light);

    // Renderer setup
    components.renderer = new THREE.WebGLRenderer();
    components.renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(components.renderer.domElement);

    // Adding components
    components.scene.add(floor);

    return components;
}

/**
 * This class creates a cube robot with a piston attached to each face used for propulsion
 * Class contains methods used for movement and propulsion.
 * Class is responsible for calculating the appropiate responses to the enviroment
 * Calculations include: Torque (Piston), Torque (Gravity), Angular Acceleration
 * Angular Velocity, Translational Acceleration, Translational Velocity
 * Each robot is assign a bounding-box used for collision detection and response
 * 
 * @param {Object} scene- Scene where robot will be placed
 * @param {Number} x - Initial x coordinate
 * @param {Number} y - Initial y coordinate
 * @param {Number} z - Initial z coordinate
 */
const GRAVITY_ACCELERATION = 9.8;

class cube_robot {
    constructor(scene, x, y, z) {
        
        this.scene = scene;

        // Instance variables
        this.time_step = 1 / 360; // Common value used in the field for physics simulations
        this.tipping_point_angle = THREE.MathUtils.degToRad(45);
        this.full_rotation = THREE.MathUtils.degToRad(90);
        this.rest_angle = THREE.MathUtils.degToRad(0);
        this.angle = this.tipping_point_angle;

        // Vectors used for physics calculations
        this.piston_output_force = new THREE.Vector3(0, 0, 0);
        this.angular_velocity = new THREE.Vector3(0, 0, 0);
        this.translational_velocity = new THREE.Vector3(0, 0, 0);
        this.gravity = new THREE.Vector3(0, GRAVITY_ACCELERATION * 10, 0);

        // Cube attributes and setup
        this.geometry = new THREE.BoxGeometry(2, 2, 2);
        this.material = new THREE.MeshBasicMaterial({ color: 0x333333 });
        this.cube = new THREE.Mesh(this.geometry, this.material);
        this.cube.position.set(x, y, z);

        // Components for collision detection and position in the world
        this.bounding_box = new THREE.Box3().setFromObject(this.cube);
        this.robot_size = new THREE.Vector3();
        this.bounding_box.getSize(this.robot_size);
        this.position_in_world = new THREE.Vector3();

        // Custom bounding boxes
        this.custom_bounding_box = {
            min: new THREE.Vector3(x - 1, y - 1, z - 1), 
            max: new THREE.Vector3(x + 1, y + 1, z + 1)
        };

        // Create cross_lines for better visual effects
        const wireframe = new THREE.WireframeGeometry(this.geometry);
        const line_material = new THREE.LineBasicMaterial({ color: 0x777777 });
        this.cross_lines = new THREE.Object3D();

        const horizontal_wireframe = wireframe.clone();
        horizontal_wireframe.rotateZ(Math.PI / 2);
        this.cross_lines.add(new THREE.LineSegments(horizontal_wireframe, line_material));

        const vertical_wireframe = wireframe.clone();
        vertical_wireframe.rotateX(Math.PI / 2);
        this.cross_lines.add(new THREE.LineSegments(vertical_wireframe, line_material));

        // Pistons setup
        this.piston_size = this.robot_size.x / 2 - 0.1;
        this.piston_face_location = this.piston_size - 0.8;
        this.axis_rotation_distance = (this.robot_size.x / 2) / 100;

        const piston_geometry = new THREE.CylinderGeometry(0.3, 0.3, 2, 32);
        const piston_material = new THREE.MeshBasicMaterial({ color: 0x000000 });

        this.piston_front = new THREE.Mesh(piston_geometry, piston_material);
        this.piston_top = this.piston_front.clone();
        this.piston_right = this.piston_front.clone();
        this.piston_bottom = this.piston_front.clone();
        this.piston_back = this.piston_front.clone();
        this.piston_left = this.piston_front.clone();

        this.piston_front.rotateX(Math.PI/2);
        this.piston_front.position.set(0, 0, this.piston_face_location);
        this.piston_top.position.set(0,this.piston_face_location,0);
        this.piston_right.rotateZ(Math.PI/2);
        this.piston_right.position.set(this.piston_face_location,0,0);
        this.piston_bottom.position.set(0,-this.piston_face_location,0);
        this.piston_back.rotateX(Math.PI/2);
        this.piston_back.position.set(0,0,-this.piston_face_location);
        this.piston_left.rotateZ(Math.PI/2);
        this.piston_left.position.set(-this.piston_face_location,0,0);

        // Adding elements scene and cube
        this.scene.add(this.cube);
        this.cube.add(this.cross_lines);
        this.cube.add(this.piston_front);
        this.cube.add(this.piston_top);
        this.cube.add(this.piston_right);
        this.cube.add(this.piston_bottom);
        this.cube.add(this.piston_back);
        this.cube.add(this.piston_left);
    }

    update_bounding_box() {
        this.bounding_box.setFromObject(this.cube);
    }
    
    update_custom_bounding_box() {
        const new_min = new THREE.Vector3(
            this.cube.position.x - this.robot_size.x / 2,
            this.cube.position.y - this.robot_size.y / 2,
            this.cube.position.z - this.robot_size.z / 2
            );
            
        const new_max = new THREE.Vector3(
            this.cube.position.x + this.robot_size.x / 2,
            this.cube.position.y + this.robot_size.y / 2,
            this.cube.position.z + this.robot_size.z / 2
            );
            
        this.custom_bounding_box.min.copy(new_min);
        this.custom_bounding_box.max.copy(new_max);
    }
    
    _calculate_physics(){
        
        // Randomness element for landing compenent
        this.random_value = (Math.random() * 0.1) - 0.09;

        // User input for mass of robot and piston output force
        this.user_piston_force = document.getElementById( "piston_output" ).value / 100;
        this.cube.mass = document.getElementById("mass").value/1000;

        // Torque due to gravity
        const grav_torqueX = 0
        const grav_torqueY = -this.cube.mass * (this.axis_rotation_distance) * GRAVITY_ACCELERATION * Math.sin(this.cube.rotation.x); 
        const grav_torqueZ = 0
        this.torque_grav = new THREE.Vector3(grav_torqueX, grav_torqueY, grav_torqueZ);

        // Torque due to piston
        const torqueX = this.user_piston_force * (this.axis_rotation_distance)  * Math.sin(this.angle);
        const torqueY = 0; 
        const torqueZ = this.user_piston_force * (this.axis_rotation_distance)  * Math.sin(this.angle);
        this.torque = new THREE.Vector3(torqueX, torqueY, torqueZ);

        // Translation
        const trans_x = this.user_piston_force * Math.cos(this.angle);
        const trans_y = 0;
        const trans_z = this.user_piston_force * Math.cos(this.angle);
        this.translation = new THREE.Vector3(trans_x, trans_y, trans_z);

        // Moment of inertia
        this.moment_of_inertia = (1/6) * this.cube.mass * ((this.robot_size.x/100) * (this.robot_size.x/100));

        // Angular acceleration
        this.angular_acceleration = (this.torque.clone().divideScalar(this.moment_of_inertia));
        this.translational_acceleration = (this.translation.clone().divideScalar(this.cube.mass));

        // Angular velocity
        this.angular_velocity.add(this.angular_acceleration.clone().multiplyScalar(this.time_step));
        this.translational_velocity.add(this.translational_acceleration.clone().multiplyScalar(this.time_step));
    }
    
    move_away() {
        
        this._calculate_physics();
    
        // Rotation and translation of cube based on calculations
        this.cube.rotation.x -= this.angular_velocity.z * this.time_step;
        this.cube.position.z -= this.translational_velocity.z;

        this.update_bounding_box(); 

        // Piston behaviour
        if (Math.abs(this.cube.rotation.x) < this.tipping_point_angle ){
            this.piston_bottom.rotation.x = this.angle;
            this.piston_bottom.position.y -= this.angular_velocity.z * this.time_step;
            this.piston_bottom.position.z -= this.translational_velocity.z * this.time_step;
            this.update_bounding_box(); 
        }

        // Tipping point check 
        if (Math.abs(this.cube.rotation.x) >= this.tipping_point_angle){
            this.angular_velocity.add(this.gravity.clone().multiplyScalar(this.time_step));
            this.cube.rotation.x -= this.angular_velocity.y * this.time_step;
            this.cube.position.z -= this.translational_velocity.z; 
            this.update_bounding_box(); 
        }

        // Rotation check and element of randomness when landing
        if (Math.abs(this.cube.rotation.x) >= this.full_rotation){
            this.angular_velocity.set(0,0,0);
            this.translational_velocity.set(0,0,0);
            this.cube.rotation.x = 0;
            this.cube.rotation.y = this.random_value;
            this.piston_bottom.position.y = -this.piston_face_location;
            this.update_bounding_box(); 
        }
        
        // Updating global position of robot after every refresh
        this.cube.getWorldPosition(this.position_in_world);
    }
    
    move_closer() {

        this._calculate_physics();
        
        // Rotation and translation of cube based on calculations
        this.cube.rotation.x += this.angular_velocity.z * this.time_step;
        this.cube.position.z += this.translational_velocity.z;

        this.update_bounding_box();

        // Piston behaviour
        if (Math.abs(this.cube.rotation.x) < this.tipping_point_angle ){
            this.piston_bottom.rotation.x = -this.angle;
            this.piston_bottom.position.y -= this.angular_velocity.z * this.time_step;
            this.piston_bottom.position.z += this.translational_velocity.z * this.time_step;
            this.update_bounding_box(); 
        }

        // Tipping point check 
        if (Math.abs(this.cube.rotation.x) >= this.tipping_point_angle){
            this.angular_velocity.add(this.gravity.clone().multiplyScalar(this.time_step));
            this.cube.rotation.x += this.angular_velocity.y * this.time_step;
            this.cube.position.z += this.translational_velocity.z;
            this.update_bounding_box(); 
        }

        // Rotation check and element of randomness when landing
        if (Math.abs(this.cube.rotation.x) >= this.full_rotation){
            this.angular_velocity.set(0,0,0);
            this.translational_velocity.set(0,0,0);
            this.cube.rotation.x = 0;
            this.cube.rotation.y = this.random_value;
            this.piston_bottom.position.y = -this.piston_face_location;
            this.update_bounding_box(); 
        }
        
        // Updating global position of robot after every refresh
        this.cube.getWorldPosition(this.position_in_world);
    }
    
    move_left() {

        this._calculate_physics();

        // Rotation and translation of cube based on calculations
        this.cube.rotation.z += this.angular_velocity.x * this.time_step;
        this.cube.position.x -= this.translational_velocity.x;

        this.update_bounding_box(); 

        // Piston behaviour
        if (Math.abs(this.cube.rotation.z) < this.tipping_point_angle ){
            this.piston_bottom.rotation.z = -this.angle;
            this.piston_bottom.position.y -= this.angular_velocity.x * this.time_step;
            this.piston_bottom.position.x -= this.translational_velocity.x * this.time_step;
            this.update_bounding_box(); 
        }

        // Tipping point check 
        if (Math.abs(this.cube.rotation.z) >= this.tipping_point_angle){
            this.angular_velocity.add(this.gravity.clone().multiplyScalar(this.time_step));
            this.cube.rotation.z += this.angular_velocity.x * this.time_step;
            this.cube.position.x -= this.translational_velocity.x;  
            this.update_bounding_box(); 
        }

        // Rotation check and element of randomness when landing
        if (Math.abs(this.cube.rotation.z) >= this.full_rotation){
            this.angular_velocity.set(0,0,0);
            this.translational_velocity.set(0,0,0);
            this.cube.rotation.z = 0;
            this.cube.rotation.y = this.random_value;
            this.piston_bottom.position.y = -this.piston_face_location;
            this.update_bounding_box(); 
        }

        // Updating global position of robot after every refresh
        this.cube.getWorldPosition(this.position_in_world);
    }
    
    move_right() {

        this._calculate_physics();

        // Rotation and translation of cube based on calculations
        this.cube.rotation.z -= this.angular_velocity.x * this.time_step;
        this.cube.position.x += this.translational_velocity.x;

        this.update_bounding_box(); 

        // Piston behaviour
        if (Math.abs(this.cube.rotation.z) < this.tipping_point_angle ){
            this.piston_bottom.rotation.z = this.angle;
            this.piston_bottom.position.y -= this.angular_velocity.x * this.time_step;
            this.piston_bottom.position.x -= this.translational_velocity.x * this.time_step;
            this.update_bounding_box(); 
        }

        // Tipping point check 
        if (Math.abs(this.cube.rotation.z) >= this.tipping_point_angle){
            this.angular_velocity.add(this.gravity.clone().multiplyScalar(this.time_step));
            this.cube.rotation.z -= this.angular_velocity.x * this.time_step;
            this.cube.position.x += this.translational_velocity.x;  
            this.update_bounding_box(); 
        }

        // Rotation check and element of randomness when landing
        if (Math.abs(this.cube.rotation.z) >= this.full_rotation){
            this.angular_velocity.set(0,0,0);
            this.translational_velocity.set(0,0,0);
            this.cube.rotation.z = 0;
            this.cube.rotation.y = this.random_value;
            this.piston_bottom.position.y = -this.piston_face_location;
            this.update_bounding_box(); 
        }

        // Updating global position of robot after every refresh
        this.cube.getWorldPosition(this.position_in_world);
    }
}

/**
 * Main collision detection and resolusion algorithms
 * @param {Array} robot_array - Array contining every cube robot
 */
function collision_detection (robot_array) {
    for (let i = 0; i < robot_array.length; i++) {
        const robot_a = robot_array[i];
        
        // Boundary checks
        if (robot_a.position_in_world.x < -20 || robot_a.position_in_world.x > 20 
            ||robot_a.position_in_world.z < -20 || robot_a.position_in_world.z > 20) {
            update_robot_index(robot_a);
            update_piston(robot_a);
            
        }

        for (let j = i + 1; j < robot_array.length; j++) {
            const robot_b = robot_array[j];
            
            if (robot_a.bounding_box.intersectsBox(robot_b.bounding_box)) {
                update_robot_index(robot_a);
                update_robot_index(robot_b);
                update_piston(robot_a);
                update_piston(robot_b);
            }
        }
    }
}

/**
 * Helper function for collision response to update robot's direction after a collision
 * @param {cube_robot} robot 
 */
function update_robot_index(robot) {
    if (robot.index === 1) {
        robot.index = 2;
    } else if (robot.index === 2) {
        robot.index = 1;
    } else if (robot.index === 3) {
        robot.index = 4;
    } else if (robot.index === 4) {
        robot.index = 3;
    }
}

/**
 * Helper function for collision respons to update right behaviour of piston after a collision 
 * @param {cube_robot} robot 
 */
function update_piston(robot) {
    if (robot.index === 1 || robot.index === 2) {
        robot.piston_bottom.rotation.x = robot.cube.rotation.x;
        robot.piston_bottom.position.y = -robot.piston_face_location;
    } else if (robot.index === 3 || robot.index === 4) {
        robot.piston_bottom.rotation.x = -robot.cube.rotation.x;
        robot.piston_bottom.position.y = robot.piston_face_location;
    }
}

/**
 * Helper function to create robots and assign them specific coordinates 
 * @param {Object} scene - Scene where robots will be placed
 * @param {Number} x - Initial x coordinate
 * @param {Number} y - Initial y coordinate
 * @param {Number} z - Initial z coordinate
 * @returns {cube_robot} - Instance of the cube_robot class
 */
function create_robots(scene, x, y, z) {
    return new cube_robot(scene, x, y, z);
}

/**
 * This function initializes the robots and place them in arbitrary position avoiding overlaps in the 3D field
 * @param {Object} scene - Scene wher robots will be place 
 * @param {Number} num_robots - Number of robots to be made
 * @returns {Array} - Array contining every cube robot
 */
function initialize_robots(scene, num_robots) {
    
    // Array holding all robots
    const robot_array = [];
    
    const movement = [1, 2, 3, 4]; 
    
    for (let i = 0; i < num_robots; i++) {
        let valid_pos = false;
        let robot;

        // Making robots within space delimiters and avoid overlap
        while (!valid_pos) {
            const x = Math.random() * 36 - 18;
            const z = Math.random() * 36 - 18;

            robot = create_robots(scene, x, 1.3, z);
            robot.name = `Robot ${i + 1}`;

            // Overlap check
            let collision_detected = false;
            for (const current_robot_array of robot_array) {
                if (robot.bounding_box.intersectsBox(current_robot_array.bounding_box)) {
                    collision_detected = true;
                    break;
                }
            }
            
            // Removing robots that overlap
            if (!collision_detected) {
                valid_pos = true;
            } else {
                scene.remove(robot.cube);
            }
        }
        
        // Randomly assign an initial direction index to each robot
        robot.index = movement[Math.floor(Math.random() * movement.length)];
        
        robot_array.push(robot);
    }
    return robot_array;
}

/**
 * This function assigns each robot's initial direction based on their indices
 * @param {Array} robot_array - Array contining every cube robot
 */
function assign_initial_direction(robot_array) {
    for (let i = 0; i < robot_array.length; i++) {
        const robot = robot_array[i];
        
        switch (robot.index) {
            case 1:
                robot.move_away();
                break;
            case 2:
                robot.move_closer();
                break;
            case 3:
                robot.move_left();
                break;
            case 4:
                robot.move_right();
                break;
            default:
                break;
            }
        }
}

/**
 * Init function for Physics Engine
 * Contains animate function, responsible for recursively updating and refresing the scene
 * @returns {Object} - An object with the animate function
 */
function init_engine() {
    const setup_components = setup_scene();
    const url_params = new URLSearchParams(window.location.search);
    const num_robots_create = url_params.get('num_robots') || 1;
    const robot_array = initialize_robots(setup_components.scene, num_robots_create);
    
    function animate() {
        requestAnimationFrame(animate);
        setup_components.renderer.render(setup_components.scene, setup_components.camera);
        collision_detection(robot_array);
        assign_initial_direction(robot_array);
    }
    return {animate};
}

/**
 * Initializes the engine and starts the animation loop.
 */
const physicsEngine = init_engine();
physicsEngine.animate();