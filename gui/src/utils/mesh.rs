use egui::{Mesh, Pos2};

fn is_point_in_triangle(p: &Pos2, a: &Pos2, b: &Pos2, c: &Pos2) -> bool {
    let v0 = *c - *a; //[c.x - a.x, c.y - a.y];
    let v1 = *b - *a; //[b.x - a.x, b.y - a.y];
    let v2 = *p - *a; //[p.x - a.x, p.y - a.y];

    let dot00 = v0.dot(v0); //v0[0] * v0[0] + v0[1] * v0[1];
    let dot01 = v0.dot(v1); //v0[0] * v1[0] + v0[1] * v1[1];
    let dot02 = v0.dot(v2); //v0[0] * v2[0] + v0[1] * v2[1];
    let dot11 = v1.dot(v1); //v1[0] * v1[0] + v1[1] * v1[1];
    let dot12 = v1.dot(v2); //v1[0] * v2[0] + v1[1] * v2[1];

    let denom = dot00 * dot11 - dot01 * dot01;
    if denom == 0.0 {
        return false; // Avoid division by zero
    }

    let u = (dot11 * dot02 - dot01 * dot12) / denom;
    let v = (dot00 * dot12 - dot01 * dot02) / denom;

    return (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0)
}

pub fn is_point_in_mesh(point: &Pos2, mesh: Mesh) -> bool {
    for triangle in mesh.indices.chunks(3) {
        if triangle.len() < 3 {
            continue;
        }

        let a = mesh.vertices[triangle[0] as usize].pos;
        let b = mesh.vertices[triangle[1] as usize].pos;
        let c = mesh.vertices[triangle[2] as usize].pos;

        if is_point_in_triangle(point, &a, &b, &c) {
            return true;
        }
    }
    
    false
}