#include "moveit_workspace.h"


void MoveItWorkSpace::init(geometry_msgs::Pose toPose, geometry_msgs::Pose fromPose, std::string frame_id, std::string robotArm_id)
{
    double c = 2;
    double k = 0.1;
    this->dx = ( abs( (toPose.position.x - fromPose.position.x) ) + k ) * c;
    this->dy = ( abs( (toPose.position.y - fromPose.position.y) ) + k ) * c;
    this->dz = ( abs( (toPose.position.z - fromPose.position.z) ) + k ) * c;

    this->frame_id = frame_id;

    this->robotArm_id = robotArm_id;
    this->box_ids = {robotArm_id + "_topBox", robotArm_id + "_bottomBox", robotArm_id + "_frontBox", robotArm_id + "_backBox", robotArm_id + "_leftBox", robotArm_id + "_rightBox"};

    averagePose.orientation.w = 1.0;
    averagePose.position.x = (toPose.position.x + fromPose.position.x)/2.0;
    averagePose.position.y = (toPose.position.y + fromPose.position.y)/2.0;
    averagePose.position.z = (toPose.position.z + fromPose.position.z)/2.0;
}


void MoveItWorkSpace::add(){
    this->planning_scene_interface.addCollisionObjects(this->workspaceBoxes);
}

void MoveItWorkSpace::remove(){
    this->planning_scene_interface.removeCollisionObjects(this->box_ids);
}

void MoveItWorkSpace::calculateWorkspace()
{
    this->workspaceBoxes = {
        this->createBox(
            this->box_ids[0],
            {this->dx, this->dy, this->thickness},
            {0.0, 0.0, this->dz/2.0 + this->thickness/2.0}
        ),
        this->createBox(
            this->box_ids[1],
            {this->dx, this->dy, this->thickness},
            {0.0, 0.0, -this->dz/2.0 - this->thickness/2.0}
        ),
        this->createBox(
            this->box_ids[2],
            {this->dx, this->thickness, this->dz},
            {0.0, this->dy/2.0 + this->thickness/2.0, 0.0}
        ),
        this->createBox(
            this->box_ids[3],
            {this->dx, this->thickness, this->dz},
            {0.0, -this->dy/2.0 - this->thickness/2.0, 0.0}
        ),
        this->createBox(
            this->box_ids[4],
            {this->thickness, this->dy, this->dz},
            {this->dx/2.0 - this->thickness/2.0, 0.0, 0.0}
        ),
        this->createBox(
            this->box_ids[5],
            {this->thickness, this->dy, this->dz},
            {-this->dx/2.0 - this->thickness/2.0, 0.0, 0.0}
        ),
    };

}

moveit_msgs::CollisionObject MoveItWorkSpace::createBox(std::string id, std::vector<double> size, std::vector<double> delta)
{
    moveit_msgs::CollisionObject collisionBox;
    collisionBox.header.frame_id = this->frame_id;
    std::cout << frame_id << std::endl;
    /* The id of the object is used to identify it. */
    collisionBox.id = id;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = size[0]; // x
    primitive.dimensions[1] = size[1]; // y
    primitive.dimensions[2] = size[2]; // z

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = this->averagePose.position.x + delta[0];
    box_pose.position.y = this->averagePose.position.y + delta[1];
    box_pose.position.z = this->averagePose.position.z + delta[2];

    collisionBox.primitives.push_back(primitive);
    collisionBox.primitive_poses.push_back(box_pose);
    collisionBox.operation = collisionBox.ADD;


    return collisionBox;
}