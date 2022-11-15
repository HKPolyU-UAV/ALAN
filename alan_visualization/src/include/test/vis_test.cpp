



// Polyhedron3D ros_to_polyhedron(const decomp_ros_msgs::Polyhedron& msg){
//   Polyhedron3D poly;
//   for(unsigned int i = 0; i < msg.points.size(); i++){
//     Vec3f pt(msg.points[i].x,
//              msg.points[i].y,
//              msg.points[i].z);
//     Vec3f n(msg.normals[i].x,
//             msg.normals[i].y,
//             msg.normals[i].z);
//     poly.add(Hyperplane3D(pt, n));
//   }
//   return poly;
// }

// vec_E<Polyhedron3D> ros_to_polyhedron_array(const decomp_ros_msgs::PolyhedronArray& msg) {
//   vec_E<Polyhedron3D> polys(msg.polyhedrons.size());

//   for(size_t i = 0; i < msg.polyhedrons.size(); i++)
//     polys[i] = ros_to_polyhedron(msg.polyhedrons[i]);

//   return polys;
// }

// decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron2D& poly){
//   decomp_ros_msgs::Polyhedron msg;
//   for (const auto &p : poly.hyperplanes()) {
//     geometry_msgs::Point pt, n;
//     pt.x = p.p_(0);
//     pt.y = p.p_(1);
//     pt.z = 0;
//     n.x = p.n_(0);
//     n.y = p.n_(1);
//     n.z = 0;
//     msg.points.push_back(pt);
//     msg.normals.push_back(n);
//   }

//   geometry_msgs::Point pt1, n1;
//   pt1.x = 0, pt1.y = 0, pt1.z = 0.01;
//   n1.x = 0, n1.y = 0, n1.z = 1;
//   msg.points.push_back(pt1);
//   msg.normals.push_back(n1);

//   geometry_msgs::Point pt2, n2;
//   pt2.x = 0, pt2.y = 0, pt2.z = -0.01;
//   n2.x = 0, n2.y = 0, n2.z = -1;
//   msg.points.push_back(pt2);
//   msg.normals.push_back(n2);

//   return msg;
// }

// decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly)
// {
//   decomp_ros_msgs::Polyhedron msg;
//   for (const auto &p : poly.hyperplanes()) {
//     geometry_msgs::Point pt, n;
//     pt.x = p.p_(0);
//     pt.y = p.p_(1);
//     pt.z = p.p_(2);
//     n.x = p.n_(0);
//     n.y = p.n_(1);
//     n.z = p.n_(2);
//     msg.points.push_back(pt);
//     msg.normals.push_back(n);
//   }

//   return msg;
// }

// template <int Dim>
// decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs){
//   decomp_ros_msgs::PolyhedronArray msg;
//   for (const auto &v : vs)
//     msg.polyhedrons.push_back(polyhedron_to_ros(v));
//   return msg;
// }

// decomp_ros_msgs::Polyhedron msg;
//     decomp_ros_msgs::PolyhedronArray poly_msg;

//     geometry_msgs::Point pt, n;


//     pt.x = 1;
//     pt.y = 1;
//     pt.z = 1;
//     n.x = 1;
//     n.y = 1;
//     n.z = 0;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = 0;
//     pt.y = 0;
//     pt.z = 1;

//     n.x = -1;
//     n.y = -1;
//     n.z = 0;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = 0;
//     pt.y = 0;
//     pt.z = 4;
//     n.x = 0;
//     n.y = 0;
//     n.z = 1;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = 0;
//     pt.y = 0;
//     pt.z = -1;
//     n.x = 0;
//     n.y = 0;
//     n.z = -1;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = 2;
//     pt.y = -2;
//     pt.z = -1;
//     n.x = 1;
//     n.y = -1;
//     n.z = 0;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = -2;
//     pt.y = 2;
//     pt.z = -1;
//     n.x = -1;
//     n.y = 1;
//     n.z = 0;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);


//     poly_msg.polyhedrons.push_back(msg);
//     poly_msg.header.frame_id = "map";
//     poly_pub.publish(poly_msg);