use std::ops::{Add, AddAssign};
use eframe::emath::Pos2;
use eframe::epaint::{Mesh, Rgba, Vertex};
use egui::{Frame, Rect, Widget};
use egui::PointerButton::Secondary;
use transform_gizmo_egui::math::{DMat4, DQuat, DVec3, DVec4, Vec2, Vec4Swizzles};
use crate::data_source::DataSource;
use std::fs::read_to_string;
use clap::builder::TypedValueParser;
use futures::AsyncReadExt;
use nalgebra::UnitQuaternion;

pub enum View{
    Texture,
    Modules,
    Data
}
pub trait Colorizer{
    fn get_color(&self,vertex:DVec3) -> [f32;4];
}
#[derive(Clone)]
pub struct ShapeObj{
    shape: Shape,
    rotation: DQuat,
    translation: DVec3,
    scale:DVec3,
    uv:Vec<u32>,
    buffer:Option<Shape>
}

impl ShapeObj {
    pub fn new(shape: Shape,
               rotation: DQuat,
               translation: DVec3,
               scale:DVec3,
               uv:Vec<u32>
    ) ->ShapeObj{
        Self{
            shape,
            uv,
            scale,
            translation,
            rotation,
            buffer:None,
        }
    }
    pub fn apply_permanent(mut self)->Self{
        self.shape = self.get_shape();
        self.translation=DVec3::ZERO;
        self.rotation=DQuat::IDENTITY;
        self.scale=DVec3::ONE;
        self.buffer=None;
        self
    }
    pub fn from_shape(s:Shape)->ShapeObj{
        Self{
            shape:s,
            rotation:DQuat::IDENTITY,
            translation: DVec3::ZERO,
            scale:DVec3::ONE,
            uv:vec![],
            buffer:None,
        }
    }
    pub fn default()-> ShapeObj{
        Self{
            shape: Shape::cube(1.0),
            rotation: Default::default(),
            translation: Default::default(),
            scale: Default::default(),
            uv: vec![],
            buffer:None
        }
    }
    pub fn get_shape(&mut self) ->Shape{
        if self.buffer.is_none(){
            let mut sh = self.shape.clone();
            let mat = DMat4::from_scale_rotation_translation(self.scale,self.rotation,self.translation);
            sh.vertices=sh.vertices.iter().map(|v| (mat * DVec4::from((*v,1.0))).xyz()).collect();
            self.buffer = Some(sh);
        }
        self.buffer.clone().unwrap()
    }
    pub fn set_scale(&mut self,axis_prop:DVec3,val:f64){
        self.scale = axis_prop*val;
        self.buffer=None;
    }
    pub fn scale(mut self,axis_prop:DVec3,val:f64)->Self{
        self.scale=self.scale+(axis_prop*val);
        self.buffer=None;
        self
    }
    pub fn set_translation(&mut self,translation:DVec3){
        self.translation = translation;
        self.buffer=None;
    }
    pub fn translate(mut self,translation:DVec3)->Self{
        self.translation+=translation;
        self.buffer=None;
        self
    }
    pub fn set_rotation(&mut self,rotation:DQuat){
        self.rotation = rotation;
        self.buffer=None;
    }
    pub fn rotate(mut self,rotation:DQuat)->Self{
        self.rotation = self.rotation*rotation;
        self.buffer=None;
        self
    }
}

#[derive(Clone)]
pub struct Shape {
    vertices:Vec<DVec3>,  // Coord
    colors:Vec<[f32;4]>,    // RGBA
    indices:Vec<u32>        // Faces
}
pub struct ObjShape{
    positions: Vec<[f32;3]>,
    uvs: Vec<[f32;3]>,
    normals: Vec<[f32;3]>,
    indices: Vec<u32>,
}
impl Shape {
    pub fn new(vertices:Vec<DVec3>, colors:Vec<[f32;4]>, indices:Vec<u32>) -> Self {
        Self{vertices, colors, indices}
    }
    pub fn none() -> Self {
        Self::new(vec![], vec![], vec![])
    }
    pub fn cube_with_color(size:f64,rgba:[f32;4]) -> Self {
        Self{
            vertices:   vec![DVec3::from([0.0,0.0,0.0]),
                             DVec3::from([size,0.0,0.0]),
                             DVec3::from([0.0,size,0.0]),
                             DVec3::from([0.0,0.0,size]),
                             DVec3::from([size,size,0.0]),
                             DVec3::from([size,0.0,size]),
                             DVec3::from([0.0,size,size]),
                             DVec3::from([size,size,size])],
            colors:     vec![rgba,
                             rgba,
                             rgba,
                             rgba,
                             rgba,
                             rgba,
                             rgba,
                             rgba],
            indices:    vec![0,1,2,1,2,4,0,1,3,3,1,5,0,2,3,2,3,6,1,4,5,4,5,7,2,4,6,4,6,7,3,5,6,5,6,7]
        }
    }
    pub fn oval_with_color(center:DVec3, rx:DVec3,ry:DVec3, n:u32, rgba:[f32;4]) -> Self {
        let mut vertices: Vec<DVec3> = Vec::with_capacity((5+n*4) as usize);
        let mut colors : Vec<[f32;4]> = Vec::with_capacity((5+n*4) as usize);
        let mut faces : Vec<u32> = Vec::with_capacity((4 * (n + 1) * 3) as usize);

        vertices.push(DVec3::from(center));
        colors.push(rgba);
        let base :[DVec3;4] = [rx,ry,-rx,-ry];
        for j in 0..4{
            vertices.push(center+base[j]);
            colors.push(rgba);
            for i in 0..n{
                let d = DVec3::from([center[0] + (base[j][0]*(n-i) as f64)/(n+1) as f64 + base[(j+1)%4][0] * (i+1) as f64 /(n+1) as f64,
                    center[1] + (base[j][1]*(n-i) as f64)/(n+1) as f64 + base[(j+1)%4][1] * (i+1) as f64 /(n+1) as f64,
                    center[2] + (base[j][2]*(n-i) as f64)/(n+1) as f64 + base[(j+1)%4][2] * (i+1) as f64 /(n+1) as f64,]);

                vertices.push(
                    (d/d.length())* ((DVec3::from(base[j]).length()*(n-i) as f64/(n+1) as f64)+(DVec3::from(base[(j+1)%4]).length()*(i+1) as f64/(n+1) as f64) )
                );
                colors.push(rgba);
            }
        }
        for i in 1..4 * (n + 1)+1{
            faces.push(0);
            faces.push(i);
            if i+1 != 4 * (n + 1)+1{
                faces.push(i+1);
            }else{
                faces.push(1);
            }
        }


        Self{
            vertices,
            colors,
            indices:faces
        }
    }

    pub fn arrow_with_color(start:DVec3, end:DVec3, size:f64, rgba:[f32;4],n:u32) -> Self {
        let z_dir : DVec3 = end-start;
        let (d_x,d_y) = z_dir.any_orthonormal_pair();
        let d_x = d_x * size;
        let d_y = d_y * size;

        //TODO add head, fix wrong vertices

        let u = Shape::oval_with_color(start,d_x,d_y,n,rgba);
        let o = Shape::oval_with_color(end,d_x,d_y,n,rgba);
        let off = u.vertices.len() as u32;
        let mut combined = u+o;
        let c = (n+1)*4;
        combined.indices.reserve((2 * c) as usize);
        for i in 0..c{ //connect bought circles
            // U
            combined.indices.push(1u32+i); // skip 0
            if i != c-1{
                combined.indices.push(i + 2u32);
            }else{
                combined.indices.push(1);
            }
            combined.indices.push(off+i+1u32); // skip 0
            //O
            combined.indices.push(off+1u32+i); // skip 0
            if i != c-1{
                combined.indices.push(off+ i + 2u32);
                combined.indices.push(i+2u32); // skip 0
            }else{
                combined.indices.push(off+1);
                combined.indices.push(1); // skip 0
            }
        }
        combined
    }
    pub fn cube(size:f64)-> Self{
        Self::cube_with_color(size,[255.0,255.0,255.0,1.0])
    }
    pub fn colorize(&mut self, colorizer: &dyn Colorizer) {
        for i in 0..self.vertices.len(){
            self.colors[i] = colorizer.get_color(self.vertices[i]);
        }
    }
    pub fn from_obj(path: &str)-> Self{ // TODO implement complexer models
        let mut indices : Vec<u32> = Vec::new();
        let mut vertices: Vec<DVec3> = Vec::new();
        let mut colors: Vec<[f32;4]> = Vec::new();

        let mut max:[f64;3] = [f64::NEG_INFINITY,  f64::NEG_INFINITY,  f64::NEG_INFINITY ];
        let mut min:[f64;3] = [f64::INFINITY,  f64::INFINITY, f64::INFINITY ];
        let mut max_len = f64::NEG_INFINITY;
        let mut error = false;

        for line in read_to_string(path).unwrap().lines() {
            let l = line.trim_ascii_end().trim_ascii_start();
            let parts:Vec<&str> = l.split_ascii_whitespace().collect();
            if parts.len() == 0{continue;}
                match parts[0] {
                    "v" => {
                        let x_w = parts[1].parse::<f64>();
                        let y_w = parts[2].parse::<f64>();
                        let z_w = parts[3].parse::<f64>();
                        if x_w.is_err() || y_w.is_err() || z_w.is_err() {
                            println!("Failed to load vertex from obj [{:?}]",line);//TODO replace with real log
                            error = true;
                            break;
                        }
                        let x = x_w.unwrap();
                        if x > max[0] { max[0] = x };
                        if x < min[0] { min[0] = x };
                        let y = y_w.unwrap();
                        if y > max[1] { max[1] = y };
                        if y < min[1] { min[1] = y };
                        let z = z_w.unwrap();
                        if z > max[2] { max[2] = z };
                        if z < min[2] { min[2] = z };
                        let vec = DVec3::from([x, y, z]);
                        let len = vec.length();
                        if len > max_len { max_len = len; }
                        vertices.push(vec);
                        colors.push([255.0, 255.0, 255.0, 1.0]);
                    },
                    "f" => {
                        let mut ind : Vec<u32> = Vec::new();
                        for i in 1..parts.len(){
                            ind.push(parts[i].split("/").collect::<Vec<_>>()[0].parse::<u32>().unwrap())
                        }
                        if ind.len() < 3{
                            println!("Failed to load face from obj [{:?}]",line);//TODO replace with real log
                            error = true;
                            break;
                        }
                        let org = 0;
                        let mut last = 1;
                        while last+1 < ind.len(){ // make face to triangles
                            indices.push(ind[org]-1);
                            indices.push(ind[last]-1);
                            indices.push(ind[last+1]-1);
                            last = last+1;
                        }
                    },
                    _ => {}
                }
        }
        if error||vertices.len()!=colors.len()||indices.len()%3!=0{
            println!("ERROR: Could not parse obj-model => fallback to cube");//TODO replace with real log
            return Self::cube(1.0)
        }
        // normalise
        let x_correction = -(min[0]+max[0])/2.0;
        let y_correction = -(min[1]+max[1])/2.0;
        let z_correction = -(min[2]+max[2])/2.0;
        vertices = vertices.iter().map(|v|{(*v+DVec3::from([x_correction,y_correction,z_correction]))/max_len}).collect();

        Self{
            vertices,
            colors,
            indices
        }
    }
}
impl AddAssign for Shape {
    fn add_assign(&mut self, rhs: Self) {
        assert_eq!(self.vertices.len(),self.colors.len());
        assert_eq!(rhs.vertices.len(),rhs.colors.len());
        assert_eq!(self.indices.len()%3,0);
        assert_eq!(rhs.indices.len()%3,0);
        let index_offset = self.vertices.len() as u32;
        self.vertices.extend(rhs.vertices);
        self.colors.extend(rhs.colors);
        self.indices
            .extend(rhs.indices.into_iter().map(|idx| index_offset + idx));
    }
}
impl Add for Shape{
    type Output = Self;
    fn add(mut self, rhs: Self) -> Self {
        self += rhs;
        self
    }
}
pub struct Camera{
    rotation: DQuat,
    translation: DVec3,
    viewport: Rect,

    view_projection: Option<DMat4>,
    projection_matrix: DMat4,
}
impl Camera {
    pub fn new(rotation:DQuat,translation:DVec3, viewport:Rect,projection_matrix:DMat4)->Self{
        Camera{
            rotation,
            translation,
            viewport,
            projection_matrix,
            view_projection: None,
        }
    }
    pub fn default(viewport:Rect)->Self{
        let d = Camera{
            rotation: DQuat::IDENTITY,
            translation: DVec3::from([0.0,5.0,0.0]),
            viewport,
            view_projection:None,
            projection_matrix: DMat4::perspective_infinite_reverse_rh(
                std::f64::consts::PI / 4.0,
                (viewport.width() / viewport.height()).into(),
                0.1,
            )
        };
        d
    }
    pub fn update_settings(&mut self,viewport:Rect,rotation:DQuat,position:DVec3){
        self.viewport = viewport;
        self.rotation = rotation;
        self.translation = position;
        self.view_projection = None;
    }
    pub fn get_view_projection(&mut self) ->DMat4{
        if self.view_projection.is_none(){
            self.projection_matrix = DMat4::perspective_infinite_reverse_rh(
                std::f64::consts::PI / 4.0,
            (self.viewport.width() / self.viewport.height()).into(),
            0.1,
            );
            let view_matrix = DMat4::look_at_rh(self.translation, DVec3::ZERO, DVec3::Z);
            self.view_projection = Some(self.projection_matrix * view_matrix);
        }
        self.view_projection.unwrap()
    }
}
pub struct Model3DState{
    disable_render: bool,
    activate_model: bool,
    activate_texture: bool,
    activate_direction: bool,
    activate_orientation: bool,
    orientation_model: Shape,
    activate_fixed_cam: bool,

    model: ShapeObj,
    scale: DVec3,
    rotation: DQuat,
    translation: DVec3,
    camera:Camera,
    projected_shape:Option<Mesh>,
}
impl Model3DState{
    pub fn default() -> Model3DState{
        Model3DState{
            disable_render:true,
            activate_model:true,
            activate_texture: true,
            activate_orientation:false,
            orientation_model:Shape::new( // compass
                vec![DVec3::from([0.0, 0.0, 0.0]), DVec3::from([0.0, 0.0, 0.0]),
                     DVec3::from([1.0, 0.0, 0.0]), DVec3::from([0.25, 0.0, 0.25]), DVec3::from([1.0, 0.0, 0.0]), DVec3::from([0.25, 0.0, -0.25]), //x
                     DVec3::from([-1.0, 0.0, 0.0]), DVec3::from([-0.25, 0.0, -0.25]), DVec3::from([-1.0, 0.0, 0.0]), DVec3::from([-0.25, 0.0, 0.25]), //-x
                     DVec3::from([0.0, 0.0, 1.0]), DVec3::from([-0.25, 0.0, 0.25]), DVec3::from([0.0, 0.0, 1.0]), DVec3::from([0.25, 0.0, 0.25]), //z
                     DVec3::from([0.0, 0.0, -1.0]), DVec3::from([0.25, 0.0, -0.25]), DVec3::from([0.0, 0.0, -1.0]), DVec3::from([-0.25, 0.0, -0.25]), //-z
                ],
                vec![[255.0,255.0,255.0,1.0],[0.0,0.0,0.0,1.0],
                     [255.0,255.0,255.0,1.0],[255.0,255.0,255.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                     [255.0,255.0,255.0,1.0],[255.0,255.0,255.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                     [255.0,255.0,255.0,1.0],[255.0,255.0,255.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                     [255.0,255.0,255.0,1.0],[255.0,255.0,255.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                ],
                vec![0,2,3,1,4,5,
                            0,6,7,1,8,9,
                            0,10,11,1,12,13,
                            0,14,15,1,16,17,
                ],
            ),
            activate_direction:false,
            activate_fixed_cam:true,

            model: ShapeObj::default(),
            scale: DVec3::ONE,
            rotation:DQuat::IDENTITY,
            translation: DVec3::ZERO,
            camera:Camera::default(Rect::ZERO),
            projected_shape:None,
        }
    }
    pub fn force_update(&mut self){
        self.projected_shape = None;
        self.camera.view_projection = None;
    }
    pub fn set_model(&mut self, model:ShapeObj){
        self.model = model;
        self.projected_shape = None;
    }

    pub(crate) fn world_projection(&mut self, mvp: DMat4, pos: DVec3) -> Option<Pos2> {
        let mut pos = mvp * DVec4::from((pos, 1.0));

        if pos.w < 1e-10 {
            return None;
        }

        pos /= pos.w;
        pos.y *= -1.0;

        let center = self.camera.viewport.center();

        Some(Pos2::new(
            (center.x as f64 + pos.x * self.camera.viewport.width() as f64 / 2.0) as f32,
            (center.y as f64 + pos.y * self.camera.viewport.height() as f64 / 2.0) as f32,
        ))
    }
}

pub struct Model3DPlot<'a>{
    state:&'a mut Model3DState,
    current_rotation:Option<UnitQuaternion<f32>>
}
impl <'a> Model3DPlot<'a>{
    pub fn new(state: &'a mut Model3DState, data_source: &mut dyn DataSource) -> Self{
        let rotation = data_source.vehicle_states()
            .rev()
            .find_map(|(_t, vs)| vs.orientation);
        Self{
            state,
            current_rotation:rotation
        }
    }
}
impl<'a> Widget for Model3DPlot<'a> {

    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let viewport = ui.max_rect();
        if self.state.camera.viewport != viewport{
            self.state.camera.viewport = viewport;
            self.state.force_update();
        }

        let response = ui.allocate_response(
            viewport.size(),
            egui::Sense::click_and_drag()
        );

        if self.state.disable_render{ // TODO add auto disable at cpu usage
            ui.label("Paused");
        }else { //maybe move rendering out of ui function
            if self.current_rotation.is_some(){ // TODO maybe add interpolation to fill missing timestamps
                let uq = self.current_rotation.unwrap(); // z_top
                let cur_rot = DQuat::from_xyzw(uq.i as f64, uq.j as f64, uq.k as f64, uq.w as f64); //
                if self.state.model.rotation!=cur_rot {
                    self.state.model.set_rotation(cur_rot);
                    self.state.force_update();
                }
            }
            if response.dragged_by(Secondary) && response.drag_delta()[0]!=0.0{
                self.state.rotation *= DQuat::from_axis_angle(DVec3::Z, (0.0001 + response.drag_delta()[0]) as f64);
                self.state.force_update();
                self.state.activate_fixed_cam=false
            }
            if self.state.activate_fixed_cam==true && self.state.rotation != DQuat::IDENTITY{
                self.state.rotation = DQuat::IDENTITY;
                self.state.force_update()
            }

            ui.input(|i| if i.key_pressed(egui::Key::ArrowLeft){self.state.rotation *= DQuat::from_axis_angle(DVec3::Z, 0.1);self.state.force_update();self.state.activate_fixed_cam=false});
            ui.input(|i| if i.key_pressed(egui::Key::ArrowRight){self.state.rotation *= DQuat::from_axis_angle(DVec3::Z, -0.1);self.state.force_update();self.state.activate_fixed_cam=false});

            let painter = ui.painter().with_clip_rect(viewport);
            if self.state.projected_shape.is_none() {
                // build scene Object
                let mut scene_shape = Shape::none();
                if self.state.activate_orientation{
                    scene_shape += self.state.orientation_model.clone();
                }
                if self.state.activate_direction{
                    let sh_x = Shape::arrow_with_color(DVec3::from([0.0,0.0,0.0]), DVec3::from([1.0,0.0,0.0]), 0.05, [255.0,0.0,0.0,1.0],1);
                    let sh_y = Shape::arrow_with_color(DVec3::from([0.0,0.0,0.0]), DVec3::from([0.0,1.0,0.0]), 0.05, [0.0,255.0,0.0,1.0],1);
                    let sh_z = Shape::arrow_with_color(DVec3::from([0.0,0.0,0.0]), DVec3::from([0.0,0.0,1.0]), 0.05, [0.0,0.0,255.0,1.0],1);
                    scene_shape += sh_x;
                    scene_shape += sh_y;
                    scene_shape += sh_z;
                }
                if self.state.activate_model{
                    let model = self.state.model.get_shape();
                    if self.state.activate_texture{
                        //TODO tessellation
                        // model = model.tessellate_with(texture)
                        //TODO fuel?
                    }
                    scene_shape += model;
                }

                // render scene Object
                let t = DMat4::from_rotation_translation(self.state.rotation.normalize(), self.state.translation);
                let m = self.state.camera.get_view_projection() * t;
                let projected_vertices = scene_shape.vertices.clone()
                    .iter()
                    .filter_map(|pos| self.state.world_projection(m, DVec3::from(*pos)))
                    .collect::<Vec<_>>();

                self.state.projected_shape = Some(Mesh {
                    indices: scene_shape.indices.clone(),
                    vertices: projected_vertices
                        .into_iter()
                        .zip(scene_shape.colors.clone())
                        .map(|(pos, [r, g, b, a])| Vertex {
                            pos: pos.into(),
                            uv: Pos2::default(),
                            color: Rgba::from_rgba_premultiplied(r, g, b, a).into(),
                        })
                        .collect(),
                    ..Default::default()
                });
            }
            painter.add(self.state.projected_shape.clone().unwrap());
        }

        // add buttons
        let map_type_rect = Rect::from_two_pos(
            viewport.left_top() + Vec2::new(10.0, 10.0),
            viewport.left_top() + Vec2::new(100.0, 40.0)
        );
        ui.put(map_type_rect, |ui: &mut egui::Ui| {
            Frame::window(ui.style()).show(ui, |ui| { // TODO add reload when button is clicked
                ui.horizontal(|ui| {
                    if self.state.disable_render{
                        ui.selectable_value(&mut self.state.disable_render, false, "▶");
                    } else {
                        ui.selectable_value(&mut self.state.disable_render, true, "⏸");
                    }
                    ui.separator();
                    ui.toggle_value(&mut self.state.activate_model, "🚀");
                    ui.toggle_value(&mut self.state.activate_texture,"🟪");
                    ui.toggle_value(&mut self.state.activate_orientation, "o");
                    ui.toggle_value(&mut self.state.activate_direction, "d");
                    ui.separator();
                    ui.toggle_value(&mut self.state.activate_fixed_cam, "🔗"); //TODO implement function
                });
            }).response
        });

        response
    }
}