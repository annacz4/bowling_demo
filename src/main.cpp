#include "glew.h"
#include "freeglut.h"
#include "glm.hpp"
#include "ext.hpp"
#include <iostream>
#include <cmath>
#include <vector>

#include "Shader_Loader.h"
#include "Render_Utils.h"
#include "Camera.h"
#include "Texture.h"
#include "Physics.h"

#include <Windows.h>

Core::Shader_Loader shaderLoader;
GLuint programColor;
GLuint programTexture;

obj::Model planeModel, pinModel, sphereModel;
GLuint boxTexture, groundTexture, ballTexture;

glm::vec3 cameraPos = glm::vec3(-1, 10, 50);
glm::vec3 cameraDir;
glm::vec3 cameraSide;
float cameraAngle = 0;
glm::mat4 cameraMatrix, perspectiveMatrix;

glm::vec3 lightDir = glm::normalize(glm::vec3(0.5, -1, -0.5));


// Initalization of physical scene (PhysX)
Physics pxScene(10.0f /* gravity (m/s^2) */);

// fixed timestep for stable and deterministic simulation
const double physicsStepTime = 1.f / 60.f;
double physicsTimeToProcess = 0;

PxRigidStatic *planeBody = nullptr;
PxMaterial *planeMaterial = nullptr;

std::vector<PxRigidDynamic*> pinBodies;
PxMaterial *pinMaterial = nullptr;

PxRigidDynamic *sphereBody = nullptr;
PxMaterial *sphereMaterial = nullptr;

PxRigidActor* kula = nullptr;

struct Renderable {
    obj::Model *model;
    glm::mat4 modelMatrix;
    GLuint textureId;
    PxVec3 position;
};
std::vector<Renderable*> renderables;

bool ballLaunched = false;

void initRenderables()
{
    /* Load models */
    planeModel = obj::loadModelFromFile("models/plane.obj");
    pinModel = obj::loadModelFromFile("models/pin.obj");
    sphereModel = obj::loadModelFromFile("models/sphere.obj");

    /* Load textures */
    groundTexture = Core::LoadTexture("textures/wood.png");
    boxTexture = Core::LoadTexture("textures/c.jpg");
    ballTexture = Core::LoadTexture("textures/b.jpg");

    /* Set ground */
    Renderable *ground = new Renderable();
    ground->model = &planeModel;
    ground->textureId = groundTexture;
    ground->modelMatrix = glm::rotate(glm::radians(90.f), glm::vec3(0.f, 0.f, 1.f));
    renderables.emplace_back(ground);

    /* Create Pins */
    for (int i = 0; i < 10; i++) {
        Renderable* pin = new Renderable();
        pin->model = &pinModel;
        pin->textureId = boxTexture;
        renderables.emplace_back(pin);
    }

    /* Create Ball */
    Renderable *sphere = new Renderable();
    sphere->model = &sphereModel;
    sphere->textureId = ballTexture;
    renderables.emplace_back(sphere);
}


/* Initialize actors */
void initPhysicsScene()
{
	planeBody = pxScene.physics->createRigidStatic(PxTransformFromPlaneEquation(PxPlane(0, 1, 0, 0)));
	planeMaterial = pxScene.physics->createMaterial(0.5, 0.5, 0.6);

	PxShape* planeShape = pxScene.physics->createShape(PxPlaneGeometry(), *planeMaterial);
    planeBody->attachShape(*planeShape);
	planeShape->release();
    planeBody->userData = renderables[0];
    pxScene.scene->addActor(*planeBody);

	pinMaterial = pxScene.physics->createMaterial(0.5, 0.5, 0.2f);

    for (int i = 0; i < 4; i++) {
        PxRigidDynamic* pinBody = nullptr;
        PxShape* pinShape = nullptr;
        pinBody = pxScene.physics->createRigidDynamic(PxTransform(PxVec3(((i % 4) - 2) * 2, 3.0f, -10)));
        pinBody->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y);
        pinShape = pxScene.physics->createShape(PxBoxGeometry(0.5f, 3.0f, 0.5f), *pinMaterial);
        pinBody->attachShape(*pinShape);
        pinShape->release();

        pinBody->userData = renderables[i + 1];
        pinBody->setName("Pin");

        renderables[i + 1]->position = PxVec3(((i % 4) - 2) * 2, 2.5f, -10.0f);
        PxRigidBodyExt::updateMassAndInertia(*pinBody, 300.0f, &PxVec3(0.0f, -0.3f, 0.0f));
        pxScene.scene->addActor(*pinBody);
        pinBodies.emplace_back(pinBody);
    }

    for (int i = 0; i < 3; i++) {
        PxRigidDynamic* pinBody = nullptr;
        PxShape* pinShape = nullptr;
        pinBody = pxScene.physics->createRigidDynamic(PxTransform(PxVec3(((i % 3) - 1.5) * 2, 3.0f, -8)));
        pinBody->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y);

        pinShape = pxScene.physics->createShape(PxBoxGeometry(0.5f, 3.0f, 0.5f), *pinMaterial);
        pinBody->attachShape(*pinShape);
        pinShape->release();

        pinBody->userData = renderables[i + 5];
        pinBody->setName("Pin");

        renderables[i + 5]->position = PxVec3(((i % 3) - 1.5) * 2, 3.0f, -8.0f);
        PxRigidBodyExt::updateMassAndInertia(*pinBody, 300.0f, &PxVec3(0.0f, -0.3f, 0.0f));
        pxScene.scene->addActor(*pinBody);
        pinBodies.emplace_back(pinBody);
    }

    for (int i = 0; i < 2; i++) {
        PxRigidDynamic* pinBody = nullptr;
        PxShape* pinShape = nullptr;
        pinBody = pxScene.physics->createRigidDynamic(PxTransform(PxVec3(((i % 2) - 1) * 2, 3.0f, -6.0f)));
        pinBody->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y);

        pinShape = pxScene.physics->createShape(PxBoxGeometry(0.5f, 3.0f, 0.5f), *pinMaterial);
        pinBody->attachShape(*pinShape);
        pinShape->release();

        pinBody->userData = renderables[i + 8];
        pinBody->setName("Pin");

        renderables[i + 8]->position = PxVec3(((i % 2) - 1) * 2, 3.0f, -6.0f);
        PxRigidBodyExt::updateMassAndInertia(*pinBody, 300.0f, &PxVec3(0.0f, -0.3f, 0.0f));
        pxScene.scene->addActor(*pinBody);
        pinBodies.emplace_back(pinBody);
    }

    for (int i = 0; i < 1; i++) {
        PxRigidDynamic* pinBody = nullptr;
        PxShape* pinShape = nullptr;
        pinBody = pxScene.physics->createRigidDynamic(PxTransform(PxVec3(-1, 3.0f, -4)));
        pinBody->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y);

        pinShape = pxScene.physics->createShape(PxBoxGeometry(0.5f, 3.0f, 0.5f), *pinMaterial);
        pinBody->attachShape(*pinShape);
        pinShape->release();

        pinBody->userData = renderables[i + 10];
        pinBody->setName("Pin");

        renderables[i + 10]->position = PxVec3(-1, 3.0f, -4.0f);
        PxRigidBodyExt::updateMassAndInertia(*pinBody, 300.0f, &PxVec3(0.0f, -0.3f, 0.0f));
        pxScene.scene->addActor(*pinBody);
        pinBodies.emplace_back(pinBody);
    }


    sphereBody = pxScene.physics->createRigidDynamic(PxTransform(-1, 3, 30));
    sphereMaterial = pxScene.physics->createMaterial(0.3, 0.3, 0.1);
    PxShape* sphereShape = pxScene.physics->createShape(PxSphereGeometry(1.0f), *sphereMaterial);
    
    sphereBody->attachShape(*sphereShape);
    PxRigidBodyExt::updateMassAndInertia(*sphereBody, 1200.0f);
    sphereShape->release();
    sphereBody->userData = renderables[renderables.size() - 1];
    sphereBody->setName("Kula");
    pxScene.scene->addActor(*sphereBody);

}

/* Reset pins */
void resetPins() {
    auto actorFlags = PxActorTypeFlag::eRIGID_DYNAMIC;
    PxU32 nbActors = pxScene.scene->getNbActors(actorFlags);
    std::vector<PxRigidActor*> actors(nbActors);
    pxScene.scene->getActors(actorFlags, (PxActor**)&actors[0], nbActors);
    for (auto actor : actors) {
        for (auto body : pinBodies) {
            body->clearForce();
            body->clearTorque();
            body->setLinearVelocity(PxVec3(0, 0, 0));
            body->setAngularVelocity(PxVec3(0, 0, 0));
        }
        if (actor->getName() != "Kula") {
            Renderable* render = (Renderable*)actor->userData;
            actor->setGlobalPose(PxTransform(render->position));

        }
    }
}


/* Check if all pins were touched */
void checkGame() {
    int counter = 0;
    for (auto body : pinBodies) {
        if (body->isSleeping()) {
            counter++;
        }
    }

    if (counter >= 10 && ballLaunched) {
        int msgID = MessageBox(NULL, (LPCWSTR)L"Koniec gry", (LPCWSTR)L"Koniec Gry", MB_ICONINFORMATION);
        switch (msgID) {
            case IDOK: {
                kula->setGlobalPose(PxTransform(-1, 0, 30));
                sphereBody->setLinearVelocity(PxVec3(0, 0, 0));
                sphereBody->setAngularVelocity(PxVec3(0, 0, 0));
                resetPins();
                ballLaunched = false;
            }
        }
    }
}

void updateTransforms() {
    auto actorFlags = PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC;
    PxU32 nbActors = pxScene.scene->getNbActors(actorFlags);
    if (nbActors)
    {
        std::vector<PxRigidActor*> actors(nbActors);
        pxScene.scene->getActors(actorFlags, (PxActor**)&actors[0], nbActors);
        for (auto actor : actors)
        {
            
            if (!actor->userData) continue;
            Renderable *renderable = (Renderable*)actor->userData;

            if (actor->getName() == "Kula") {
                kula = actor;
            }

            PxMat44 transform = actor->getGlobalPose();
            auto &c0 = transform.column0;
            auto &c1 = transform.column1;
            auto &c2 = transform.column2;
            auto& c3 = transform.column3;

            renderable->modelMatrix = glm::mat4(
                c0.x, c0.y, c0.z, c0.w,
                c1.x, c1.y, c1.z, c1.w,
                c2.x, c2.y, c2.z, c2.w,
                c3.x, c3.y, c3.z, c3.w);
        }
    }

    checkGame();

}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        /* Move ball left */
        case 'a': {
            PxVec3 transform;
            PxTransform pos = kula->getGlobalPose();
            transform = pos.transform(PxVec3(-0.1f, 0.0f, 0.0f));
            kula->setGlobalPose(PxTransform(transform));
            break;
        }
        /* Move ball right */
        case 'd': {
            PxVec3 transform;
            PxTransform pos = kula->getGlobalPose();
            transform = pos.transform(PxVec3(0.1f, 0.0f, 0.0f));
            kula->setGlobalPose(PxTransform(transform));
            break;
        }
        /* Reseting positions */
        case 'r': {
            kula->setGlobalPose(PxTransform(-1, 0, 30));
            sphereBody->setLinearVelocity(PxVec3(0, 0, 0));
            sphereBody->setAngularVelocity(PxVec3(0, 0, 0));
            resetPins();
            ballLaunched = false;
            break;
        }
        /* Launching ball */
        case 'e': {
            sphereBody->setLinearVelocity(PxVec3(0, 0, -50));
            for (auto body : pinBodies) {
                body->wakeUp();
                body->setWakeCounter(1);
            }
            ballLaunched = true;
            break;
        }
    }
}

void mouse(int x, int y)
{
}

glm::mat4 createCameraMatrix()
{
    cameraDir = glm::normalize(glm::vec3(cosf(cameraAngle - glm::radians(90.0f)), 0, sinf(cameraAngle - glm::radians(90.0f))));
    glm::vec3 up = glm::vec3(0, 1, 0);
    cameraSide = glm::cross(cameraDir, up);

    return Core::createViewMatrix(cameraPos, cameraDir, up);
}

void drawObjectColor(obj::Model * model, glm::mat4 modelMatrix, glm::vec3 color)
{
    GLuint program = programColor;

    glUseProgram(program);

    glUniform3f(glGetUniformLocation(program, "objectColor"), color.x, color.y, color.z);
    glUniform3f(glGetUniformLocation(program, "lightDir"), lightDir.x, lightDir.y, lightDir.z);

    glm::mat4 transformation = perspectiveMatrix * cameraMatrix * modelMatrix;
    glUniformMatrix4fv(glGetUniformLocation(program, "modelViewProjectionMatrix"), 1, GL_FALSE, (float*)&transformation);
    glUniformMatrix4fv(glGetUniformLocation(program, "modelMatrix"), 1, GL_FALSE, (float*)&modelMatrix);

    Core::DrawModel(model);

    glUseProgram(0);
}

void drawObjectTexture(obj::Model * model, glm::mat4 modelMatrix, GLuint textureId)
{
    GLuint program = programTexture;

    glUseProgram(program);

    glUniform3f(glGetUniformLocation(program, "lightDir"), lightDir.x, lightDir.y, lightDir.z);
    Core::SetActiveTexture(textureId, "textureSampler", program, 0);

    glm::mat4 transformation = perspectiveMatrix * cameraMatrix * modelMatrix;
    glUniformMatrix4fv(glGetUniformLocation(program, "modelViewProjectionMatrix"), 1, GL_FALSE, (float*)&transformation);
    glUniformMatrix4fv(glGetUniformLocation(program, "modelMatrix"), 1, GL_FALSE, (float*)&modelMatrix);

    Core::DrawModel(model);

    glUseProgram(0);
}

void renderScene()
{
    double time = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    static double prevTime = time;
    double dtime = time - prevTime;
    prevTime = time;

    if (dtime < 1.f) {
        physicsTimeToProcess += dtime;
        while (physicsTimeToProcess > 0) {
            pxScene.step(physicsStepTime);
            physicsTimeToProcess -= physicsStepTime;
        }
    }

    cameraMatrix = createCameraMatrix();
    perspectiveMatrix = Core::createPerspectiveMatrix();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.1f, 0.3f, 1.0f);

    updateTransforms();

    for (Renderable* renderable : renderables) {
        drawObjectTexture(renderable->model, renderable->modelMatrix, renderable->textureId);
    }

    glutSwapBuffers();
}

void init()
{
    srand(time(0));
    glEnable(GL_DEPTH_TEST);
    programColor = shaderLoader.CreateProgram("shaders/shader_color.vert", "shaders/shader_color.frag");
    programTexture = shaderLoader.CreateProgram("shaders/shader_tex.vert", "shaders/shader_tex.frag");

    initRenderables();
    initPhysicsScene();
}

void shutdown()
{
    shaderLoader.DeleteProgram(programColor);
    shaderLoader.DeleteProgram(programTexture);
}

void idle()
{
    glutPostRedisplay();
}

int main(int argc, char ** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(200, 200);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Krêgle");
    glewInit();

    init();
    glutKeyboardFunc(keyboard);
    glutPassiveMotionFunc(mouse);
    glutDisplayFunc(renderScene);
    glutIdleFunc(idle);

    glutMainLoop();

    shutdown();

    return 0;
}
