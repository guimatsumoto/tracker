#ifndef __VIEWER_INCLUDE__
#define __VIEWER_INCLUDE__

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <unistd.h>


#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glut.h>   /* OpenGL Utility Toolkit header */

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <Eigen/Dense>
#include "structures.hpp"

class GLObject {
public:
    GLObject(Eigen::Vector3f position, bool isStatic);
    ~GLObject();

    void addPoint(float x, float y, float z, float r, float g, float b);
    void pushToGPU();
    void clear();

    void setDrawingType(GLenum type);

    void draw();

    void translate(const Eigen::Vector3f& t);
    void setPosition(const Eigen::Vector3f& p);

    void setRT(const Eigen::Affine3f& mRT);

    void rotate(const Eigen::Quaternionf& rot);
    void rotate(const Eigen::Matrix3f& m);
    void setRotation(const Eigen::Quaternionf& rot);
    void setRotation(const Eigen::Matrix3f& m);

    const Eigen::Vector3f& getPosition() const;

    Eigen::Affine3f getModelMatrix() const;

    std::vector<float> m_vertices_;
    std::vector<float> m_colors_;
    std::vector<unsigned int> m_indices_;

    bool isStatic_;

    GLenum drawingType_;
    GLuint vaoID_;
    /* Vertex buffer IDs:
     - [0]: vertices coordinates;
     - [1]: RGB color values;
     - [2]: indices;*/
    GLuint vboID_[3];

    Eigen::Vector3f position_;
    Eigen::Quaternionf rotation_;
};

class MeshObject {
    GLuint vaoID_;
    GLuint vboID_[2];
    int current_fc;
    GLenum drawingType_;


public:
    MeshObject();
    ~MeshObject();
    Eigen::Vector3f clr;

    void updateMesh(std::vector<Eigen::Vector3f> &vert, std::vector<tracker::uint3> &tri, GLenum type);
    void draw();
};

class PeoplesObject {
    GLuint vaoID_;
    GLuint vboID_[3];
    int current_fc;
    std::mutex mtx;
    std::vector<Eigen::Vector3f> vert;
    std::vector<Eigen::Vector3f> clr;
    std::vector<unsigned int> m_indices_;
    bool update;
public:
    PeoplesObject();
    ~PeoplesObject();
    //sl::float3 clr;

    void updateMesh();

    void cpy(PeoplesObject &other) {
        mtx.lock();
        vert = other.vert;
        clr = other.clr;
        m_indices_ = other.m_indices_;
        update = other.update;
        mtx.unlock();
    }

    void clear() {
        mtx.lock();
        vert.clear();
        m_indices_.clear();
        mtx.unlock();
    }

	size_t getNVert() {
		return vert.size();
	}

    void setVert(std::vector<Eigen::Vector3f> &vertices, std::vector<Eigen::Vector3f> &clr);

    void draw();
};

class LineObject {
    GLuint vaoID_;
    GLuint vboID_[2];
    int current_fc;
    std::mutex mtx;
    std::vector<Eigen::Vector3f> vert;
    std::vector<unsigned int> m_indices_;
    bool update;
public:
    LineObject();
    ~LineObject();
    Eigen::Vector3f clr;

    void updateMesh();

    void clear() {
        mtx.lock();
        vert.clear();
        m_indices_.clear();
        mtx.unlock();
    }


    void setVert(std::vector<Eigen::Vector3f> &vertices);

    void draw();
};

class GLViewer;

class PointObject {
    friend GLViewer;
    GLuint vaoID_;
    GLuint vboID_[3];
    int current_fc;
    std::mutex mtx;
    std::vector<Eigen::Vector3f> vert;
    std::vector<Eigen::Vector3f> clr;
    std::vector<unsigned int> m_indices_;
    bool update;
public:
    PointObject();
    ~PointObject();

    void updateMesh();

    void cpy(PointObject &other) {
        mtx.lock();
        vert = other.vert;
        clr = other.clr;
        m_indices_ = other.m_indices_;
        update = other.update;
        mtx.unlock();
    }

    void setVert(std::vector<Eigen::Vector3f> &pts, std::vector<Eigen::Vector3f> &clr);

    void draw();
};

class NormObject {
    GLuint vaoID_;
    GLuint vboID_[2];
    int current_fc;
    GLenum drawingType_;

    Eigen::Vector3f pts[2];
    uint2 line;

public:
    NormObject();
    ~NormObject();
    Eigen::Vector3f clr;

    void updateNorm(Eigen::Vector3f &vert, Eigen::Vector3f &normal);
    void draw();
};

class MeshTextureObject {
    GLuint vaoID_;
    GLuint vboID_[3];
    int current_fc;

public:
    MeshTextureObject();
    ~MeshTextureObject();

    void loadData(std::vector<Eigen::Vector3f> &vert, std::vector<Eigen::Vector2f> &uv, std::vector<tracker::uint3> &tri);

    void draw();

    //texture_mat texture;
};


#ifndef M_PI
#define M_PI 3.141592653
#endif


#define SAFE_DELETE( res ) if( res!=NULL )  { delete res; res = NULL; }

#define MOUSE_R_SENSITIVITY 0.015f
#define MOUSE_UZ_SENSITIVITY 0.75f
#define MOUSE_DZ_SENSITIVITY 1.25f
#define MOUSE_T_SENSITIVITY 0.1f
#define KEY_T_SENSITIVITY 0.1f

class CameraGL {
public:

    CameraGL() {

    }

    enum DIRECTION {
        UP, DOWN, LEFT, RIGHT, FORWARD, BACK
    };

    CameraGL(Eigen::Vector3f position, Eigen::Vector3f direction, Eigen::Vector3f vertical = Eigen::Vector3f(0, 1, 0)); // vertical = Eigen::Vector3f(0, 1, 0)
    ~CameraGL();

    void update();
    void setProjection(float horizontalFOV, float verticalFOV, float znear, float zfar);
    const Eigen::Affine3f& getViewProjectionMatrix() const;

    float getHorizontalFOV() const;
    float getVerticalFOV() const;

    /*
            Set an offset between the eye of the camera and its position.
            Note: Useful to use the camera as a trackball camera with z>0 and x = 0, y = 0.
            Note: coordinates are in local space.
     */
    void setOffsetFromPosition(const Eigen::Vector3f& offset);
    const Eigen::Vector3f& getOffsetFromPosition() const;

    void setDirection(const Eigen::Vector3f& direction, const Eigen::Vector3f &vertical);
    void translate(const Eigen::Vector3f& t);
    void setPosition(const Eigen::Vector3f& p);
    void rotate(const Eigen::Quaternionf& rot);
    void rotate(const Eigen::Matrix3f& m);
    void setRotation(const Eigen::Quaternionf& rot);
    void setRotation(const Eigen::Matrix3f& m);

    const Eigen::Vector3f& getPosition() const;
    const Eigen::Vector3f& getForward() const;
    const Eigen::Vector3f& getRight() const;
    const Eigen::Vector3f& getUp() const;
    const Eigen::Vector3f& getVertical() const;
    float getZNear() const;
    float getZFar() const;

    static const Eigen::Vector3f ORIGINAL_FORWARD;
    static const Eigen::Vector3f ORIGINAL_UP;
    static const Eigen::Vector3f ORIGINAL_RIGHT;

    Eigen::Affine3f projection_;
private:
    void updateVectors();
    void updateView();
    void updateVPMatrix();

    Eigen::Vector3f offset_;
    Eigen::Vector3f position_;
    Eigen::Vector3f forward_;
    Eigen::Vector3f up_;
    Eigen::Vector3f right_;
    Eigen::Vector3f vertical_;

    Eigen::Quaternionf rotation_;

    Eigen::Affine3f view_;
    Eigen::Affine3f vpMatrix_;
    float horizontalFieldOfView_;
    float verticalFieldOfView_;
    float znear_;
    float zfar_;
};

class Shader {
public:

    Shader() {
    }
    Shader(GLchar* vs, GLchar* fs);
    ~Shader();

    GLuint getProgramId();

    static const GLint ATTRIB_VERTICES_POS = 0;
    static const GLint ATTRIB_COLOR_POS = 1;
private:
    bool compile(GLuint &shaderId, GLenum type, GLchar* src);
    GLuint verterxId_;
    GLuint fragmentId_;
    GLuint programId_;
};

/*
 * This class manages the window, input events and Opengl rendering pipeline
 */
class GLViewer {
public:
    GLViewer();
    ~GLViewer();
    void exit();
    bool isEnded();
    bool isInitialized();
    void init(bool useTexture = false);

    void loadTexture();

    void update(PeoplesObject &people);
    void update(PointObject &pc);

private:
    std::mutex mtx_people;
    LineObject camRepere;
    LineObject grill;
    PeoplesObject peopleObj;
    PointObject pointcloudObj;


    /*
  Initialize OpenGL context and variables, and other Viewer's variables
     */
    void initialize();
    /*
      Rendering loop method called each frame by glutDisplayFunc
     */
    void render();
    /*
      Everything that needs to be updated before rendering must be done in this method
     */
    void update();
    /*
      Once everything is updated, every renderable objects must be drawn in this method
     */
    void draw();
    /*
      Clear and refresh inputs' data
     */
    void clearInputs();

    static GLViewer* currentInstance_;

    //! Glut Functions CALLBACKs
    static void drawCallback();
    static void mouseButtonCallback(int button, int state, int x, int y);
    static void mouseMotionCallback(int x, int y);
    static void reshapeCallback(int width, int height);
    static void keyPressedCallback(unsigned char c, int x, int y);
    static void keyReleasedCallback(unsigned char c, int x, int y);
    static void idle();

    bool ended_;

    // color settings
    float cr;
    float cg;
    float cb;

    // window size
    int wnd_w;
    int wnd_h;

    bool useTexture_;

    enum MOUSE_BUTTON {
        LEFT = 0,
        MIDDLE = 1,
        RIGHT = 2,
        WHEEL_UP = 3,
        WHEEL_DOWN = 4
    };

    enum KEY_STATE {
        UP = 'u',
        DOWN = 'd',
        FREE = 'f'
    };

    bool mouseButton_[3];
    int mouseWheelPosition_;
    int mouseCurrentPosition_[2];
    int mouseMotion_[2];
    int previousMouseMotion_[2];
    KEY_STATE keyStates_[256];

    CameraGL camera_;
    Shader shader_, shader_pc, shader_people;
    GLuint shMVPMatrixLoc_;
    GLuint shMVPMatrixLoc_pc;
    GLuint shMVPMatrixLoc_people;
    GLuint shColorLoc_;
    Eigen::Vector2i res;
    CUcontext ctx;
    bool initialized_;
};

#endif /* __VIEWER_INCLUDE__ */
