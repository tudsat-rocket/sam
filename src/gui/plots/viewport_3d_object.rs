use std::f32::consts::PI;
use std::fs::{File, read_to_string};
use std::ops::Add;
use eframe::egui;
use eframe::egui::{Color32, Mesh, Painter, Pos2, Response, Sense, Shape, Stroke, TextureHandle, TextureId, Ui, Vec2};
use nalgebra;

// Render Viewport
pub struct RenderViewport<'a>{
    viewport_size: Vec2,
    object_to_display : &'a mut Object3D,
    camera : &'a mut Camera
}

impl<'a> RenderViewport<'a> {
    pub fn new(object_to_display :&'a mut Object3D,camera :&'a mut Camera) -> RenderViewport<'a>{
        RenderViewport{
            object_to_display,
            camera,
            viewport_size:Vec2::new(480.0,480.0)
        }
    }

}
impl egui::Widget for RenderViewport<'_>{
    fn ui(mut self, ui: &mut Ui) -> Response {
        println!("Repaint: {:?}",self.camera.position);
        self.camera.render_object(ui.painter(), &mut self.object_to_display);
        ui.allocate_response(self.viewport_size,Sense::hover())
    }
}

// Camera
pub struct Camera{
    render_settings : RenderSettings,
    resolution : Vec2,
    projector : Box<dyn VertexProjector>,
    mapped_pos:Vec<Pos2>,
    position:nalgebra::Point3<f32>,
    rotation:nalgebra::Rotation3<f32>,
    update_required:bool
}

impl Default for Camera{
    fn default() -> Self {
        Camera{
            render_settings: RenderSettings{
                base_coordinate_system:Some((Color32::RED,Color32::GREEN,Color32::BLUE)),
                object_texture:None,
                object_base:None,
                vertex:Some(Color32::WHITE),
                wireframe:Some(Color32::GREEN)
            },
            resolution: Vec2::new(1080.0,1080.0),
            projector: Box::new(TotumProjector{near_plane:0.1,far_plane:100.0,h_fov:70.0,v_fov:70.0}),
            //projector: Box::new(MathProjector{}),
            mapped_pos: Vec::new(),
            position: nalgebra::Point3::new(0.0,0.0,0.0),
            rotation: nalgebra::Rotation3::from_euler_angles(0.0,0.0,0.0),
            update_required:true
        }
    }
}

impl Camera {
    pub fn set_render_settings(&mut self,new_settings:RenderSettings){
        self.render_settings = new_settings;
        self.update_required = true;
    }
    pub fn set_resolution(&mut self,new_resolution:Vec2){
        self.resolution = new_resolution;
        self.update_required = true;
    }
    pub fn set_projector(&mut self,new_projector : Box<dyn VertexProjector>){
        self.projector = new_projector;
        self.update_required = true;
    }

}
impl Camera {
    fn render_object(&mut self, painter: &Painter,mut object :&mut Object3D){
        if object.changed || self.update_required { // recalculate view
            self.update_vertices(object);
        }

        let tex_id = if self.render_settings.object_texture.is_some(){ self.render_settings.object_texture.clone().unwrap().id()} else {TextureId::Managed(0)};
        let mut mesh = Mesh::with_texture(tex_id);
        for i in 0..self.mapped_pos.len()-4 {
            mesh.colored_vertex(*self.mapped_pos.get(i).unwrap(),self.render_settings.object_base.unwrap_or(Color32::TRANSPARENT));
        }
        for face in &object.faces {
            if face.len()==3 {
                mesh.add_triangle(*face.get(0).unwrap(),*face.get(1).unwrap(),*face.get(2).unwrap());
            }else if face.len()==4 {
                mesh.add_triangle(*face.get(0).unwrap(),*face.get(1).unwrap(),*face.get(2).unwrap());
                mesh.add_triangle(*face.get(3).unwrap(),*face.get(1).unwrap(),*face.get(2).unwrap());
            }else{
                eprintln!("Object face has not 3 or 4 vertexes => can not display");
            }
        }
        let shape = Shape::Mesh(mesh);
        painter.add(shape);
        if self.render_settings.wireframe.is_some(){
            for face in &object.faces {
                let mut face_vertexes : Vec<Pos2> = Vec::new();
                for j in 0..face.len() {
                    let pos = self.mapped_pos.get(*face.get(j).unwrap() as usize).unwrap().clone();
                    face_vertexes.push(pos);
                }

                let mut last : Pos2 = face_vertexes.last().unwrap().clone();
                for vertex in face_vertexes.clone() {
                    painter.line_segment([last,vertex],Stroke::new(2.0,self.render_settings.wireframe.unwrap()));
                    last = vertex;
                }
            }
        }
        if self.render_settings.vertex.is_some() {
            let vertex_color = self.render_settings.vertex.unwrap();
            for i in 0..self.mapped_pos.len()-4 {
                painter.circle(*self.mapped_pos.get(i).unwrap(),1.0,vertex_color,Stroke::NONE);
            }
        }

        // draw local axis
        if self.render_settings.base_coordinate_system.is_some() {
            let colors :(Color32,Color32,Color32) = self.render_settings.base_coordinate_system.unwrap();
            painter.line_segment([*self.mapped_pos.get(self.mapped_pos.len()-4).unwrap(),*self.mapped_pos.get(self.mapped_pos.len()-3).unwrap()],Stroke::new(1.5,colors.0));
            painter.line_segment([*self.mapped_pos.get(self.mapped_pos.len()-4).unwrap(),*self.mapped_pos.get(self.mapped_pos.len()-2).unwrap()],Stroke::new(1.5,colors.1));
            painter.line_segment([*self.mapped_pos.get(self.mapped_pos.len()-4).unwrap(),*self.mapped_pos.get(self.mapped_pos.len()-1).unwrap()],Stroke::new(1.5,colors.2));
        }
    }
    fn update_vertices(&mut self, mut object: &mut Object3D){
        let mut vertices : Vec<nalgebra::Vector3<f32>> = object.vertices.clone();
        // Add local system vertices
        vertices.push(nalgebra::Vector3::new(0.0,0.0,0.0));
        vertices.push(nalgebra::Vector3::new(1.0,0.0,0.0));
        vertices.push(nalgebra::Vector3::new(0.0,1.0,0.0));
        vertices.push(nalgebra::Vector3::new(0.0,0.0,1.0));
        let world_transformation = self.world_transformation_matrix();

        let mut transformed_vertices : Vec<nalgebra::Vector4<f32>> = Vec::with_capacity(vertices.len());
        for vertex in vertices {
            let vertex: nalgebra::Vector3<f32> = object.rotation * vertex;
            let vertex : nalgebra::Vector3<f32>  = object.position.coords + vertex;
            let vertex: nalgebra::Vector4<f32> = nalgebra::Vector4::new(
                vertex.x,
                vertex.y,
                vertex.z,
                1.0
            );
            let vertex = world_transformation * vertex;
            transformed_vertices.push(vertex);
        }

        self.mapped_pos = self.projector.project(self,&transformed_vertices);
        self.update_required = false;
        object.changed = false;
    }
    fn world_transformation_matrix(&self) -> nalgebra::Matrix4<f32>{
        let right = self.vec_right();
        let forward = self.vec_forward();
        let up = self.vec_up();

        nalgebra::Matrix4::new(
            right.x,up.x,forward.x,self.position.x,
            right.y,up.y,forward.y,self.position.y,
            right.z,up.z,forward.z,self.position.z,
            0.0,0.0,0.0,1.0
        )
    }
    fn vec_up(&self) -> nalgebra::Vector3<f32>{
        let default: nalgebra::Vector3<f32> = nalgebra::Vector3::new(0.0,1.0,0.0);
        self.rotation.transform_vector(&default)
    }
    fn vec_forward(&self) -> nalgebra::Vector3<f32>{
        let default: nalgebra::Vector3<f32> = nalgebra::Vector3::new(0.0,0.0,1.0);
        self.rotation.transform_vector(&default)
    }
    fn vec_right(&self) -> nalgebra::Vector3<f32>{
        let default: nalgebra::Vector3<f32> = nalgebra::Vector3::new(1.0,0.0,0.0);
        self.rotation.transform_vector(&default)
    }
    pub fn move_forward(&mut self, length:f32){
        let mut dir = self.vec_forward().clone();
        dir.set_magnitude(length);
        self.position += dir;
        self.update_required = true;
    }
    pub fn move_up(&mut self,length:f32){
        let mut dir = self.vec_up().clone();
        dir.set_magnitude(length);
        self.position += dir;
        self.update_required = true;
    }
    pub fn move_right(&mut self,length:f32){
        let mut dir = self.vec_right().clone();
        dir.set_magnitude(length);
        self.position += dir;
        self.update_required = true;
    }
}

// renderer
#[derive(Clone)]
pub struct RenderSettings{
    wireframe: Option<Color32>,
    object_base: Option<Color32>,
    object_texture: Option<TextureHandle>,
    vertex: Option<Color32>,
    base_coordinate_system: Option<(Color32, Color32, Color32)>,
}

impl RenderSettings {
    pub fn default() -> RenderSettings{
        Self::object_base_only(Color32::RED)
    }
    pub fn none() -> RenderSettings{
        RenderSettings{
            base_coordinate_system:None,
            wireframe:None,
            object_base:None,
            object_texture:None,
            vertex:None
        }
    }
    pub fn wireframe_only(wireframe_color:Color32) -> RenderSettings{
        RenderSettings{
            wireframe:Some(wireframe_color),
            .. Self::none()
        }
    }
    pub fn texture_only(texture: TextureHandle) -> RenderSettings{
        RenderSettings{
            object_texture:Some(texture),
            .. Self::none()
        }
    }
    pub fn object_base_only(object_base_color:Color32) -> RenderSettings{
        RenderSettings{
            object_base:Some(object_base_color),
            .. Self::none()
        }
    }
    pub fn vertex_only(vertex_color:Color32) -> RenderSettings{
        RenderSettings{
            vertex:Some(vertex_color),
            .. Self::none()
        }
    }

}

// Projector

pub trait VertexProjector{
    fn project(&self, camera:&Camera,vecs_to_project :&Vec<nalgebra::Vector4<f32>>) -> Vec<Pos2>;
}
#[derive(Clone)]
pub struct MathProjector{}
impl VertexProjector for MathProjector {
    fn project(&self, camera : &Camera, vectors_to_project :&Vec<nalgebra::Vector4<f32>>) -> Vec<Pos2>{
        let mut res = Vec::with_capacity(vectors_to_project.len());
        //let camera_matrix: nalgebra::Matrix4<f32> = camera.world_transformation_matrix();
        for vertex in vectors_to_project { //TODO use gpu or pp?
            //let vertex = camera_matrix * vertex; // switch?
            let vertex = nalgebra::Vector4::new(vertex.x+(vertex.z/2.0),vertex.y+(vertex.z/2.0),0.0 ,1.0);
            let vertex_pos: Pos2 = Pos2::new(vertex.x*(camera.resolution.x/2.0)+(camera.resolution.x/2.0),vertex.y*(camera.resolution.y/2.0)+(camera.resolution.y/2.0));
            //println!("V:{:?}",vertex);
            res.push(vertex_pos);
        }
        res
    }
}

pub struct TotumProjector{
    near_plane: f32,
    far_plane: f32,
    h_fov:f32,
    v_fov:f32,
}
impl TotumProjector {
    fn projection_matrix(&self) -> nalgebra::Matrix4<f32>{
        let near = self.near_plane;
        let far = self.far_plane;
        let right = (self.h_fov.to_radians()/2.0).tan();
        let left = -right;
        let top = (self.v_fov.to_radians()/2.0).tan();
        let bottom = -top;

        let m00 = 2.0/(right - left);
        let m11 = 2.0/(top - bottom);
        let m22 = (far + near)/(far - near);
        let m32 = -2.0 * near * far / (far - near);

        nalgebra::Matrix4::new(
            m00,0.0,0.0,0.0,
            0.0,m11,0.0,0.0,
            0.0,0.0,m22,1.0,
            0.0,0.0,m32,0.0
        );

        let mut matrix = [[0.0; 4]; 4];
        let aspect_ratio = 1.0 as f32 / 1.0 as f32;
        let fov_rad: f32 = 1.0 / (self.h_fov * 0.5 / 180.0 * PI).tan();

        if self.far_plane < self.near_plane {
            panic!("The view limit must be bigger than the screen position, the Z-axis direction is away from the screen");
        }
        let distance = self.far_plane - self.near_plane;

        matrix[0][0] = aspect_ratio * fov_rad;
        matrix[1][1] = fov_rad;
        matrix[2][2] = self.far_plane * distance;
        matrix[3][2] = (-self.far_plane * self.near_plane) / distance;
        matrix[2][3] = 1.0;

        nalgebra::Matrix4::from_data(nalgebra::ArrayStorage(matrix))
    }
}
impl VertexProjector for TotumProjector{
    fn project(&self,camera : &Camera,vectors_to_project :&Vec<nalgebra::Vector4<f32>>) -> Vec<Pos2>{

        let mut res = Vec::with_capacity(vectors_to_project.len());
        let projection_matrix = self.projection_matrix();

        //TODO implement out of screen mapping?
        /*
        if vertex.x.abs()>2.0||vertex.y.abs()>2.0{ // filter out of screen
            //vertex = nalgebra::Vector4::new(-1.0,-1.0,-1.0,-1.0);
        }
         */

        for vertex in vectors_to_project { //TODO use gpu or pp?
            let vertex = projection_matrix * vertex;
            let mut vertex = nalgebra::Vector4::new(vertex.x/vertex.w, vertex.y/vertex.w, vertex.z/vertex.w, 1.0);
            let vertex_pos: Pos2 = Pos2::new(vertex.x*(camera.resolution.x/2.0)+(camera.resolution.x/2.0),vertex.y*(camera.resolution.y/2.0)+(camera.resolution.y/2.0));
            res.push(vertex_pos);
        }

        res
    }
}

// 3d Object
#[derive(Clone)]
pub struct Object3D{
    position:   nalgebra::Point3<f32>,
    rotation:   nalgebra::Matrix3<f32>,
    vertices:   Vec<nalgebra::Vector3<f32>>,
    faces:      Vec<Vec<u32>>,
    changed:    bool
}

impl Default for Object3D {
    fn default() -> Self {
        Object3D{
            position:   nalgebra::Point3::new(0.0,0.0,0.0),
            rotation:   nalgebra::Matrix3::new(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0),
            vertices:   Vec::new(),
            faces:      Vec::new(),
            changed:    true
        }
    }
}

impl Object3D {
    pub fn init() -> Object3D{
        let mut vertexes : Vec<nalgebra::Vector3<f32>> = Vec::new();
        vertexes.push(nalgebra::Vector3::new(0.0,0.0,0.0));
        vertexes.push(nalgebra::Vector3::new(0.0,1.0,0.0));
        vertexes.push(nalgebra::Vector3::new(1.0,1.0,0.0));
        vertexes.push(nalgebra::Vector3::new(1.0,0.0,0.0));
        vertexes.push(nalgebra::Vector3::new(0.0,0.0,1.0));
        vertexes.push(nalgebra::Vector3::new(0.0,1.0,1.0));
        vertexes.push(nalgebra::Vector3::new(1.0,1.0,1.0));
        vertexes.push(nalgebra::Vector3::new(1.0,0.0,1.0));

        let mut faces : Vec<Vec<u32>> = Vec::new();
        faces.push(Vec::from([0,1,2,3]));
        faces.push(Vec::from([4,5,6,7]));
        faces.push(Vec::from([0,4,5,1]));
        faces.push(Vec::from([2,3,7,6]));
        faces.push(Vec::from([1,2,6,5]));
        faces.push(Vec::from([0,3,7,4]));

        Self::from_data(vertexes,faces)
    }
    pub fn from_data(vertices:Vec<nalgebra::Vector3<f32>>,faces: Vec<Vec<u32>>) -> Object3D{
        Object3D{
            vertices,
            faces,
            .. Self::default()
        }
    }
    pub fn norm(&mut self){
        let mut max:&f32 = &0.0f32;
        let binding = (self.vertices.clone());
        for vertex in &binding{
            if vertex.x > *max { max = &vertex.x }
            if vertex.y > *max { max = &vertex.y }
            if vertex.z > *max { max = &vertex.z }
        }
        for i in 0..self.vertices.len(){
            let vert = self.vertices.get_mut(i).unwrap();
            vert.x = vert.x/max;
            vert.y = vert.y/max;
            vert.z = vert.z/max;
        }

    }
    pub fn set_position(&mut self,x:f32,y:f32,z:f32){
        self.position = nalgebra::Point3::new(x,y,z);
        self.changed = true;
    }
    pub fn set_rotation(&mut self,rotation3: nalgebra::Rotation3<f32>){
        self.rotation = *rotation3.matrix();
        self.changed = true;
    }
    pub fn add_rotation(&mut self,additional_rotation_matrix: nalgebra::Matrix3<f32>){
        self.rotation *= additional_rotation_matrix;
        self.changed = true;
    }

    pub fn scale(&mut self,factors:(f32,f32,f32)){
        let scale_matrix  = MatrixProvider::scaling_matrix(nalgebra::Vector3::new(factors.0,factors.1,factors.2));
        let mut new_vertices : Vec<nalgebra::Vector3<f32>> = Vec::with_capacity(self.vertices.len());
        for vertex in &self.vertices {
            let vertex4 = nalgebra::Vector4::new(vertex.x,vertex.y,vertex.z,0.0);
            let scaled_vertex = scale_matrix * vertex4;
            new_vertices.push(scaled_vertex.xyz());
        }
        self.vertices = new_vertices;
        self.changed = true;
    }

    pub fn rotate_roll(&mut self,degrees_angle : f32){
        self.add_rotation(*nalgebra::Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(self.direction_right()),degrees_angle.to_radians()).matrix());
    }
    pub fn rotate_pitch(&mut self,degrees_angle : f32){ // TODO fix rotation mistake
        self.add_rotation(*nalgebra::Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(self.direction_up()),degrees_angle.to_radians()).matrix());
    }
    pub fn rotate_yaw(&mut self,degrees_angle : f32){
        self.add_rotation(*nalgebra::Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(self.direction_front()),degrees_angle.to_radians()).matrix());
    }

    pub fn from_obj_file(obj_file_path: &str) -> Object3D{
        let mut vertexes:Vec<nalgebra::Vector3<f32>> = Vec::new();
        let mut faces = Vec::new();
        if File::open(obj_file_path).is_err() {
            println!("ERROR: Cant open file");
            return Self::init();
        }
        //Fill data
        'line: for line in read_to_string(obj_file_path).unwrap().lines(){
            let line = line.trim_start();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if line.starts_with("v ") {
                let mut parts = line.split_whitespace();

                parts.next(); // skip "v "
                let x = parts.next().unwrap().parse();
                let y = parts.next().unwrap().parse();
                let z = parts.next().unwrap().parse();
                if x.is_err() || y.is_err() || z.is_err() {
                    continue; // skip entry
                }
                vertexes.push(
                    nalgebra::Vector3::new(x.unwrap(),y.unwrap(),z.unwrap())
                );
            }else if line.starts_with("f ") {
                let mut parts = line.split_whitespace().peekable();
                parts.next(); // skip "f "
                let mut face_vex:Vec<u32> = Vec::new();
                while parts.peek().is_some() {
                    let mut vertex_index_split = parts.next().unwrap().split('/');
                    let index_res  = vertex_index_split.next().unwrap().parse();
                    if index_res.is_err() {
                        continue 'line;
                    }else{
                        let index : u32 = index_res.unwrap();
                        face_vex.push(index-1);//obj starts with 1
                    }
                }
                faces.push(face_vex);

            }else if line.starts_with("vt ") {
                //todo!("support vertex texture coordinate")
            }
        }

        Object3D::from_data(vertexes,faces)
    }

    fn direction_front(&self) -> nalgebra::Vector3<f32>{
        let dir = nalgebra::Vector3::new(0.0,0.0,1.0);
        self.rotation * dir
    }
    fn direction_up(&self) -> nalgebra::Vector3<f32>{
        let dir = nalgebra::Vector3::new(0.0,1.0,0.0);
        self.rotation * dir
    }
    fn direction_right(&self) -> nalgebra::Vector3<f32>{
        let dir = nalgebra::Vector3::new(1.0,0.0,0.0);
        self.rotation * dir
    }

    pub fn move_object(&mut self,pos_change : nalgebra::Vector3<f32> ){
        self.position += pos_change;
        self.changed = true;
    }

    pub fn move_forward(&mut self, length:f32){
        let mut dir  = self.direction_front();
        dir.set_magnitude(length);
        self.move_object(dir);
    }
    pub fn move_up(&mut self,length:f32){
        let mut dir  = self.direction_up();
        dir.set_magnitude(length);
        self.move_object(dir);
    }
    pub fn move_right(&mut self,length:f32){
        let mut dir  = self.direction_right();
        dir.set_magnitude(length);
        self.move_object(dir);
    }
}


// Matrix provider
struct MatrixProvider{}
impl MatrixProvider {
    pub fn translation_matrix(position:nalgebra::Point3<f32>)->nalgebra::Matrix4<f32>{
        nalgebra::Matrix4::new(
            1.0,0.0,0.0,position.x,
            0.0,1.0,0.0,position.y,
            0.0,0.0,1.0,position.z,
            0.0,0.0,0.0,1.0
        )
    }
    pub fn scaling_matrix(scale:nalgebra::Vector3<f32>)->nalgebra::Matrix4<f32>{
        nalgebra::Matrix4::new(
            scale.x,0.0,0.0,0.0,
            0.0,scale.y,0.0,0.0,
            0.0,0.0,scale.z,0.0,
            0.0,0.0,0.0,1.0
        )
    }
    pub fn rotation_matrix_x(degree:f32)->nalgebra::Matrix4<f32>{
        let x_rot_radians:f32 = degree.to_radians();
        nalgebra::Matrix4::new(
            1.0,0.0,0.0,0.0,
            0.0,x_rot_radians.cos(),x_rot_radians.sin(),0.0,
            0.0,-x_rot_radians.sin(),x_rot_radians.cos(),0.0,
            0.0,0.0,0.0,1.0
        )
    }
    pub fn rotation_matrix_y(degree:f32)->nalgebra::Matrix4<f32>{
        let y_rot_radians:f32 = degree.to_radians();
        nalgebra::Matrix4::new(
            y_rot_radians.cos(),0.0,-y_rot_radians.sin(),0.0,
            0.0,1.0,0.0,0.0,
            y_rot_radians.sin(),0.0,y_rot_radians.cos(),0.0,
            0.0,0.0,0.0,1.0
        )
    }
    pub fn rotation_matrix_z(degree:f32)->nalgebra::Matrix4<f32>{
        let z_rot_radians:f32 = degree.to_radians();
        nalgebra::Matrix4::new(
            z_rot_radians.cos(),z_rot_radians.sin(),0.0,0.0,
            -z_rot_radians.sin(),z_rot_radians.cos(),0.0,0.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0
        )
    }
    pub fn screen_resolution_mapping_matrix(resolution:Vec2)->nalgebra::Matrix4<f32>{
        nalgebra::Matrix4::new(
            -resolution.x/2.0,0.0,0.0,0.0,
            0.0,-resolution.y/2.0,0.0,0.0, // 1=-
            0.0,0.0,1.0,0.0,
            resolution.x/2.0,resolution.y/2.0,0.0,1.0
        )
    }
}