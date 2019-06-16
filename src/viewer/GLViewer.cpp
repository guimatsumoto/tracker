#include "viewer/GLViewer.hpp"

GLchar* VERTEX_SHADER =
        "#version 330 core\n"
        "layout(location = 0) in vec3 in_Vertex;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "uniform vec3 u_color;\n"
        "out vec3 b_color;\n"
        "void main() {\n"
        "   b_color = u_color;\n"
        "	gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);\n"
        "}";

GLchar* VERTEX_SHADER_CLR =
        "#version 330 core\n"
        "layout(location = 0) in vec3 in_Vertex;\n"
        "layout(location = 1) in vec3 in_Color;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec3 b_color;\n"
        "void main() {\n"
        "   b_color = in_Color;\n"
        "	gl_Position = u_mvpMatrix * vec4(in_Vertex.xyz, 1);\n"
        "}";

GLchar* FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec3 b_color;\n"
        "layout(location = 0) out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = vec4(b_color, 1);\n"
        "}";

GLchar* FRAGMENT_SHADER_PC =
        "#version 330 core\n"
        "in vec3 b_color;\n"
        "layout(location = 0) out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = vec4(b_color, 0.85);\n"
        "}";

GLchar* VERTEX_SHADER_TEXTURE =
        "#version 330 core\n"
        "layout(location = 0) in vec3 in_Vertex;\n"
        "layout(location = 1) in vec2 in_UVs;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec2 UV;\n"
        "void main() {\n"
        "   gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);\n"
        "    UV = in_UVs;\n"
        "}\n";

GLchar* FRAGMENT_SHADER_TEXTURE =
        "#version 330 core\n"
        "in vec2 UV;\n"
        "uniform sampler2D texture_sampler;\n"
        "void main() {\n"
        "    gl_FragColor = vec4(texture(texture_sampler, UV).rgb, 1.0);\n"
        "}\n";

GLchar* POINTCLOUD_VERTEX_SHADER =
        "#version 330 core\n"
        "layout(location = 0) in vec4 in_VertexRGBA;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec3 b_color;\n"
        "vec4 packFloatToVec4i(const float value)\n"
        "{\n"
        "  const vec4 bitSh = vec4(256.0*256.0*256.0, 256.0*256.0, 256.0, 1.0);\n"
        "  const vec4 bitMsk = vec4(0.0, 1.0/256.0, 1.0/256.0, 1.0/256.0);\n"
        "  vec4 res = fract(value * bitSh);\n"
        "  res -= res.xxyz * bitMsk;\n"
        "  return res;\n"
        "}\n"
        "vec4 decomposeFloat(const in float value)\n"
        "{\n"
        "   uint rgbaInt = floatBitsToUint(value);\n"
        "	uint bIntValue = (rgbaInt / 256U / 256U) % 256U;\n"
        "	uint gIntValue = (rgbaInt / 256U) % 256U;\n"
        "	uint rIntValue = (rgbaInt) % 256U; \n"
        "	return vec4(rIntValue / 255.0f, gIntValue / 255.0f, bIntValue / 255.0f, 1.0); \n"
        "}\n"
        "void main() {\n"
        // Decompose the 4th channel of the XYZRGBA buffer to retrieve the color of the point (1float to 4uint)
        "   b_color = decomposeFloat(in_VertexRGBA.a).xyz;\n"
        "	gl_Position = u_mvpMatrix * vec4(in_VertexRGBA.xyz, 1);\n"
        "}";

GLchar* POINTCLOUD_FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec3 b_color;\n"
        "layout(location = 0) out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = vec4(b_color, 1);\n"
        "}";



GLObject::GLObject(Eigen::Vector3f position, bool isStatic) : isStatic_(isStatic) {
    vaoID_ = 0;
    drawingType_ = GL_TRIANGLES;
    position_ = position;
    rotation_ = Eigen::Quaternionf();
    rotation_.setIdentity();
}

GLObject::~GLObject() {
    if (vaoID_ != 0) {
        glDeleteBuffers(3, vboID_);
        glDeleteVertexArrays(1, &vaoID_);
    }
}

void GLObject::addPoint(float x, float y, float z, float r, float g, float b) {
    m_vertices_.push_back(x);
    m_vertices_.push_back(y);
    m_vertices_.push_back(z);
    m_colors_.push_back(r);
    m_colors_.push_back(g);
    m_colors_.push_back(b);
    m_indices_.push_back(m_indices_.size());
}

void GLObject::pushToGPU() {
    if (!isStatic_ || vaoID_ == 0) {
        if (vaoID_ == 0) {
            glGenVertexArrays(1, &vaoID_);
            glGenBuffers(3, vboID_);
        }
        glBindVertexArray(vaoID_);
        glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
        glBufferData(GL_ARRAY_BUFFER, m_vertices_.size() * sizeof (float), &m_vertices_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

        glBindBuffer(GL_ARRAY_BUFFER, vboID_[1]);
        glBufferData(GL_ARRAY_BUFFER, m_colors_.size() * sizeof (float), &m_colors_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_COLOR_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_COLOR_POS);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[2]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices_.size() * sizeof (unsigned int), &m_indices_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);

        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

void GLObject::clear() {
    m_vertices_.clear();
    m_colors_.clear();
    m_indices_.clear();
}

void GLObject::setDrawingType(GLenum type) {
    drawingType_ = type;
}

void GLObject::draw() {
    glBindVertexArray(vaoID_);
    glDrawElements(drawingType_, (GLsizei) m_indices_.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void GLObject::translate(const Eigen::Vector3f& t) {
    position_ = position_ + t;
}

void GLObject::setPosition(const Eigen::Vector3f& p) {
    position_ = p;
}

void GLObject::setRT(const Eigen::Affine3f& mRT) {
    position_ = mRT.translation();
    rotation_ = Eigen::Quaternionf(mRT.rotation());
}

void GLObject::rotate(const Eigen::Quaternionf& rot) {
    rotation_ = rot * rotation_;
}

void GLObject::rotate(const Eigen::Matrix3f& m) {
    this->rotate(Eigen::Quaternionf(m));
}

void GLObject::setRotation(const Eigen::Quaternionf& rot) {
    rotation_ = rot;
}

void GLObject::setRotation(const Eigen::Matrix3f& m) {
    this->setRotation(Eigen::Quaternionf(m));
}

const Eigen::Vector3f& GLObject::getPosition() const {
    return position_;
}

Eigen::Affine3f GLObject::getModelMatrix() const {
    Eigen::Affine3f m(Eigen::Affine3f::Identity());
    Eigen::Quaternionf quat(rotation_);
    quat.normalize();
    Eigen::Matrix3f associated_rotation = quat.toRotationMatrix();
    m.rotate(associated_rotation);
    m.pretranslate(position_);
    return m;
}

MeshObject::MeshObject() {
    current_fc = 0;
    vaoID_ = 0;
}

MeshObject::~MeshObject() {
    current_fc = 0;
    if (vaoID_)
        glDeleteBuffers(2, vboID_);
}

void MeshObject::updateMesh(std::vector<tracker::float3> &vert, std::vector<tracker::uint3> &tri, GLenum type) {
    if (vaoID_ == 0) {
        glGenVertexArrays(1, &vaoID_);
        glGenBuffers(2, vboID_);
    }
    drawingType_ = type;
    glBindVertexArray(vaoID_);

    glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
    glBufferData(GL_ARRAY_BUFFER, vert.size() * sizeof (tracker::float3), vert.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, tri.size() * sizeof (tracker::uint3), &tri[0], GL_DYNAMIC_DRAW);

    current_fc = tri.size() * 3;

    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void MeshObject::draw() {
    if ((current_fc > 0) && (vaoID_ > 0)) {
        glBindVertexArray(vaoID_);
        glDrawElements(drawingType_, (GLsizei) current_fc, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

PeoplesObject::PeoplesObject() {
    current_fc = 0;
    vaoID_ = 0;
    update = false;
}

PeoplesObject::~PeoplesObject() {
    current_fc = 0;
    if (vaoID_)
        glDeleteBuffers(3, vboID_);
}

void PeoplesObject::setVert(std::vector<tracker::float3> &vertices, std::vector<tracker::float3> &clr) {
    mtx.lock();
#if 0
    for (unsigned i = 0; i < vertices.size(); i+=2){
        std::cout << vertices[i].x << " "
                  << vertices[i].y << " "
                  << vertices[i].c << std::endl;
        std::cout << vertices[i+1].x << " "
                  << vertices[i+1].y << " "
                  << vertices[i+1].c << std::endl;
    }
#endif
    vert = vertices;
    this->clr = clr;

    m_indices_.clear();
    m_indices_.reserve(vertices.size());
    for (int i = 0; i < vertices.size(); i++)
        m_indices_.push_back(i);

    update = true;
    mtx.unlock();
}

void PeoplesObject::updateMesh() {
    if (update) {
        mtx.lock();

        if (vaoID_ == 0) {
            glGenVertexArrays(1, &vaoID_);
            glGenBuffers(3, vboID_);
        }

        glBindVertexArray(vaoID_);

        glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
        //glBufferData(GL_ARRAY_BUFFER, vert.size() * sizeof (tracker::float3), &vert[0], GL_DYNAMIC_DRAW);
        glBufferData(GL_ARRAY_BUFFER, vert.size() * sizeof (tracker::float3), vert.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

        glBindBuffer(GL_ARRAY_BUFFER, vboID_[1]);
        glBufferData(GL_ARRAY_BUFFER, clr.size() * sizeof (tracker::float3), clr.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_COLOR_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_COLOR_POS);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[2]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices_.size() * sizeof (unsigned int), &m_indices_[0], GL_DYNAMIC_DRAW);

        current_fc = (int) m_indices_.size();

        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        mtx.unlock();
    }
    update = false;
}

void PeoplesObject::draw() {
    if ((current_fc > 0) && (vaoID_ > 0) && mtx.try_lock()) {
        glBindVertexArray(vaoID_);
        glLineWidth(4);
        glDrawElements(GL_LINES, (GLsizei) current_fc, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        mtx.unlock();
    }
}

LineObject::LineObject() {
    current_fc = 0;
    vaoID_ = 0;
    update = false;
}

LineObject::~LineObject() {
    current_fc = 0;
    if (vaoID_)
        glDeleteBuffers(2, vboID_);
}

//void LineObject::setVert(std::vector<Eigen::Vector3f> &vertices) {
void LineObject::setVert(std::vector<tracker::float3> &vertices){
    mtx.lock();
    vert = vertices;

    m_indices_.clear();
    m_indices_.reserve(vertices.size());
    for (int i = 0; i < vertices.size(); i++)
        m_indices_.push_back(i);

    update = true;
    mtx.unlock();
}

void LineObject::updateMesh() {
    if (update) {
        mtx.lock();
        if (vaoID_ == 0) {
            glGenVertexArrays(1, &vaoID_);
            glGenBuffers(2, vboID_);
        }

        glBindVertexArray(vaoID_);

        glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
        //glBufferData(GL_ARRAY_BUFFER, vert.size() * sizeof (tracker::float3), &vert[0], GL_DYNAMIC_DRAW);
        glBufferData(GL_ARRAY_BUFFER, vert.size() * sizeof (tracker::float3), vert.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[1]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices_.size() * sizeof (unsigned int), &m_indices_[0], GL_DYNAMIC_DRAW);

        current_fc = (int) m_indices_.size();

        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        mtx.unlock();
    }
    update = false;
}

void LineObject::draw() {
    if ((current_fc > 0) && (vaoID_ > 0) && mtx.try_lock()) {
        glBindVertexArray(vaoID_);
        glLineWidth(1);
        glDrawElements(GL_LINES, (GLsizei) current_fc, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        mtx.unlock();
    }
}

//_________________________________________________

PointObject::PointObject() {
    current_fc = 0;
    vaoID_ = 0;
    update = false;
}

PointObject::~PointObject() {
    current_fc = 0;
    if (vaoID_)
        glDeleteBuffers(3, vboID_);
}

void PointObject::setVert(std::vector<tracker::float3> &pts, std::vector<tracker::float3> &clr_) {
    mtx.lock();

    vert = pts;
    clr = clr_;

    if (m_indices_.size() != vert.size()) {
        m_indices_.resize(vert.size());
        for (int i = 0; i < vert.size(); i++)
            m_indices_[i] = i;
    }

    update = true;
    mtx.unlock();
}

void PointObject::updateMesh() {
    if (update) {
        mtx.lock();

        if (vaoID_ == 0) {
            glGenVertexArrays(1, &vaoID_);
            glGenBuffers(3, vboID_);
        }

        glBindVertexArray(vaoID_);

        glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
        glBufferData(GL_ARRAY_BUFFER, vert.size() * sizeof (tracker::float3), vert.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

        glBindBuffer(GL_ARRAY_BUFFER, vboID_[1]);
        glBufferData(GL_ARRAY_BUFFER, clr.size() * sizeof (tracker::float3), clr.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_COLOR_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_COLOR_POS);


        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[2]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices_.size() * sizeof (unsigned int), &m_indices_[0], GL_DYNAMIC_DRAW);

        current_fc = (int) m_indices_.size();
        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        mtx.unlock();
    }
    update = false;
}

void PointObject::draw() {
    if ((current_fc > 0) && (vaoID_ > 0) && mtx.try_lock()) {
        glBindVertexArray(vaoID_);
        glDrawElements(GL_POINTS, (GLsizei) current_fc, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        mtx.unlock();
    }
}

//_________________________________________________

NormObject::NormObject() {
    current_fc = 0;
    vaoID_ = 0;
}

NormObject::~NormObject() {
    current_fc = 0;
    if (vaoID_)
        glDeleteBuffers(2, vboID_);
}

inline Eigen::Vector3f applyRot(Eigen::Vector3f &pt, Eigen::Affine3f &rot) {
    Eigen::Vector3f tmp;
    // Extract rotation
    Eigen::Matrix3f associated_rotation = rot.rotation();
    tmp[0] = pt[0] * rot(0,0) + pt[1] * rot(0,1) + pt[2] * rot(0,2);
    tmp[1] = pt[0] * rot(1,0) + pt[1] * rot(1,1) + pt[2] * rot(1,2);
    tmp[2] = pt[0] * rot(2,0) + pt[1] * rot(2,1) + pt[2] * rot(2,2);
    return tmp;
}

// Never called
void NormObject::updateNorm(Eigen::Vector3f &vert, Eigen::Vector3f &normal) {
/*
    if (vaoID_ == 0) {
        glGenVertexArrays(1, &vaoID_);
        glGenBuffers(2, vboID_);
    }

    glBindVertexArray(vaoID_);

    pts[0] = vert;

    Eigen::Vector3f t_(1, 0, 0);
    Eigen::Matrix3f rot_x(normal.x, t_);
    t_ = Eigen::Vector3f(0, 1, 0);
    Eigen::Matrix3f rot_y(normal.y, t_);
    t_ = translation_mat(0, 0, 1);
    rotation_mat rot_z(normal.z, t_);

    float3 pt(1, 1, 1);
    pt += vert;
    pt = applyRot(pt, rot_x);
    pt = applyRot(pt, rot_y);
    pt = applyRot(pt, rot_z);

    pts[0] = vert;
    pts[1] = pt;

    line.x = 0;
    line.y = 1;

    glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
    glBufferData(GL_ARRAY_BUFFER, 2 * sizeof (float3), &pts[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof (uint2), &line[0], GL_DYNAMIC_DRAW);

    current_fc = 2;

    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
*/
}

void NormObject::draw() {
    if ((current_fc > 0) && (vaoID_ > 0)) {
        glBindVertexArray(vaoID_);
        glDrawElements(GL_LINES, (GLsizei) current_fc, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

MeshTextureObject::MeshTextureObject() {
    current_fc = 0;
    vaoID_ = 0;
}

MeshTextureObject::~MeshTextureObject() {
    current_fc = 0;
}

void MeshTextureObject::loadData(std::vector<tracker::float3> &vert, std::vector<tracker::float2> &uv, std::vector<tracker::uint3> &tri) {
    if (vaoID_ == 0) {
        glGenVertexArrays(1, &vaoID_);
        glGenBuffers(3, vboID_);
    }

    glBindVertexArray(vaoID_);

    glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
    glBufferData(GL_ARRAY_BUFFER, vert.size() * sizeof (tracker::float3), vert.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

    glBindBuffer(GL_ARRAY_BUFFER, vboID_[1]);
    glBufferData(GL_ARRAY_BUFFER, uv.size() * sizeof (tracker::float2), uv.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(Shader::ATTRIB_COLOR_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(Shader::ATTRIB_COLOR_POS);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[2]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, tri.size() * sizeof (tracker::uint3), tri.data(), GL_DYNAMIC_DRAW);

    current_fc = tri.size() * 3;

    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void MeshTextureObject::draw() {
    if ((current_fc > 0) && (vaoID_ > 0)) {
        glBindVertexArray(vaoID_);
        glDrawElements(GL_TRIANGLES, (GLsizei) current_fc, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}


GLViewer* GLViewer::currentInstance_ = nullptr;

void getColor(int num_segments, int i, float &c1, float &c2, float &c3) {
    float r = fabs(1. - (float(i)*2.) / float(num_segments));
    c1 = (0.1 * r);
    c2 = (0.3 * r);
    c3 = (0.8 * r);
}

GLViewer::GLViewer() : initialized_(false) {
    if (currentInstance_ != nullptr) {
        delete currentInstance_;
    }
    currentInstance_ = this;

    wnd_w = 1000;
    wnd_h = 1000;

    cb = 0.847058f;
    cg = 0.596078f;
    cr = 0.203921f;

    mouseButton_[0] = mouseButton_[1] = mouseButton_[2] = false;

    clearInputs();
    previousMouseMotion_[0] = previousMouseMotion_[1] = 0;
    ended_ = true;

}

GLViewer::~GLViewer() {
}

void GLViewer::exit() {
    //printf("GLViewer::exit()\n");
    if (initialized_) {
        ended_ = true;
        glutLeaveMainLoop();
	    // Kill CUDA talvez precisara manter...
	    //cuCtxDestroy(ctx);
	    cuDevicePrimaryCtxRelease(0);
    }
}

bool GLViewer::isEnded() {
    return ended_;
}

void GLViewer::init(bool useTexture) {
    useTexture_ = useTexture;
    res = Eigen::Vector2i(1280, 720);
    cuDevicePrimaryCtxRetain(&ctx, 0);
    //cuCtxGetCurrent(&ctx);
    initialize();
    //wait for OpenGL stuff to be initialized
    while (!isInitialized()) usleep(1000);
}

void GLViewer::initialize() {
    char *argv[1];
    argv[0] =(char*)'\0';
    int argc = 1;
    glutInit(&argc, argv);
    glutInitWindowSize(wnd_w, wnd_h);
    glutInitWindowPosition(900, 40);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("3D Pose Viewer");

    GLenum err = glewInit();
    if (GLEW_OK != err)
        std::cout << "ERROR: glewInit failed with error: " << glewGetErrorString(err) << "\n";

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_CLAMP);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Compile and create the shader
    shader_ = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
    shader_people = Shader(VERTEX_SHADER_CLR, FRAGMENT_SHADER);
    shader_pc = Shader(VERTEX_SHADER_CLR, FRAGMENT_SHADER_PC);

    //printf("base shader program ID: %d\n", shader_.getProgramId());

    shMVPMatrixLoc_ = glGetUniformLocation(shader_.getProgramId(), "u_mvpMatrix");
    //printf("glGetUniformLocation from base shader: %d\n", (int)shMVPMatrixLoc_);
    shMVPMatrixLoc_pc = glGetUniformLocation(shader_pc.getProgramId(), "u_mvpMatrix");
    shMVPMatrixLoc_people = glGetUniformLocation(shader_people.getProgramId(), "u_mvpMatrix");
    shColorLoc_ = glGetUniformLocation(shader_.getProgramId(), "u_color");

    // Create the camera
    camera_ = CameraGL(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, -1));
    camera_.setOffsetFromPosition(Eigen::Vector3f(0, 0, 0));
    //Eigen::Matrix3f cam_rot;
    // Get associated rotation matrix from euler angle (180, 0, 0)
    // Simulate Euler angle with AngleAxis - bora testar, sepa nao funciona...
    Eigen::AngleAxisf xangle(M_PI, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yangle(0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zangle(0, Eigen::Vector3f::UnitZ());
    //Eigen::Matrix3f m = zangle * yangle * xangle;
    //Eigen::Quaternionf q(m);
    Eigen::Quaternion<float> q = xangle * yangle * zangle;
    //cam_rot = q.matrix();

    camera_.setRotation(q);

    // Map glut function on this class methods
    glutDisplayFunc(GLViewer::drawCallback);
    glutMouseFunc(GLViewer::mouseButtonCallback);
    glutMotionFunc(GLViewer::mouseMotionCallback);
    glutReshapeFunc(GLViewer::reshapeCallback);
    glutKeyboardFunc(GLViewer::keyPressedCallback);
    glutKeyboardUpFunc(GLViewer::keyReleasedCallback);

    //std::vector<Eigen::Vector3f> grillvec;
    std::vector<tracker::float3> grillvec;
    float span = 20.f;
    for (int i = (int) -span; i <= (int) span; i++) {
        //grillvec.emplace_back();
        //grillvec.back() = Eigen::Vector3f((float)i, .0, (float)-span);
        //grillvec.back() = tracker::float3{(float)i, .0, (float)-span};
        grillvec.push_back(tracker::float3{(float)i, .0, (float)-span});

        //grillvec.emplace_back();
        //grillvec.back() = Eigen::Vector3f((float)i, .0, (float)span);
        //grillvec.back() = tracker::float3{(float)i, .0, (float)span};
        grillvec.push_back(tracker::float3{(float)i, .0, (float)span});

        //grillvec.emplace_back();
        //grillvec.back() = Eigen::Vector3f((float)-span, .0, (float)i);
        //grillvec.back() = tracker::float3{(float)-span, .0, (float)i};
        grillvec.push_back(tracker::float3{(float)-span, .0, (float)i});

        //grillvec.emplace_back();
        //grillvec.back() = Eigen::Vector3f((float)span, .0, (float)i);
        //grillvec.back() = tracker::float3{(float)span, .0, (float)i};
        grillvec.push_back(tracker::float3{(float)span, .0, (float)i});
    }
    grill.clr = Eigen::Vector3f(.33, .33, .33);
    //tracker::float3 color = {.33, .33, .33};
    //grill.clr = color;
    grill.setVert(grillvec);
    grill.updateMesh();

    initialized_ = true;
    ended_ = false;
    //printf("Finished GLViewer initialization\n");
}

void GLViewer::update(PeoplesObject &people) {
    //printf("Entrou no GLViewer::update()\n");
    mtx_people.lock();
    peopleObj.cpy(people);
    mtx_people.unlock();
    //printf("Saiu do GLViewer::update()\n");
}

void GLViewer::update(PointObject &pc_) {
    mtx_people.lock();
    pointcloudObj.cpy(pc_);
    mtx_people.unlock();
}

void GLViewer::render() {
    //printf("render()\n");
    if (!ended_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glClearColor(0.12, 0.12, 0.12, 1.0f);
        //glLineWidth(2);
        glPointSize(1);
        update();
        mtx_people.lock();
        draw();
        mtx_people.unlock();
        glutSwapBuffers();
        usleep(10000);
        glutPostRedisplay();
    }
}

bool GLViewer::isInitialized() {
    return initialized_;
}

void GLViewer::update() {
    //printf("GLViewer::update()\n");
    if (keyStates_['q'] == KEY_STATE::UP || keyStates_['Q'] == KEY_STATE::UP || keyStates_[27] == KEY_STATE::UP) {
        currentInstance_->exit();
        return;
    }

    // Rotation of the camera
    if (mouseButton_[MOUSE_BUTTON::LEFT]) {
	    // Create proper quaternion for camera rotation 1
	    Eigen::Vector3f axis = camera_.getRight();
	    Eigen::AngleAxisf angle_axis(mouseMotion_[1] * MOUSE_R_SENSITIVITY, axis.normalized());
        Eigen::Quaternionf camera_rot_quat(angle_axis);
        camera_.rotate(camera_rot_quat);
	    // Create proper quaternion for camera rotation 2
	    axis = camera_.getVertical() * -1;
	    //axis.normalize();
	    angle_axis = Eigen::AngleAxisf(mouseMotion_[0] * MOUSE_R_SENSITIVITY, axis.normalized());
	    camera_rot_quat = Eigen::Quaternionf(angle_axis);
        camera_.rotate(camera_rot_quat);
    }

    // Translation of the camera on its plane
    if (mouseButton_[MOUSE_BUTTON::RIGHT]) {
        camera_.translate(camera_.getUp() * (float) mouseMotion_[1] * MOUSE_T_SENSITIVITY);
        camera_.translate(camera_.getRight() * (float) mouseMotion_[0] * MOUSE_T_SENSITIVITY);
    }

    // Zoom of the camera
#if 1
    if (mouseWheelPosition_ != 0) {
        float distance = Eigen::Vector3f(camera_.getOffsetFromPosition()).norm();
        if (mouseWheelPosition_ > 0 && distance > camera_.getZNear()) { // zoom
            camera_.setOffsetFromPosition(camera_.getOffsetFromPosition() * MOUSE_UZ_SENSITIVITY);
        } else if (distance < camera_.getZFar()) {// unzoom
            camera_.setOffsetFromPosition(camera_.getOffsetFromPosition() * MOUSE_DZ_SENSITIVITY);
        }
    }
#else
    if (mouseWheelPosition_ != 0) {
        float distance = Eigen::Vector3f(camera_.getOffsetFromPosition()).norm();
        if (mouseWheelPosition_ > 0 && distance > camera_.getZNear()) { // zoom
            camera_.setOffsetFromPosition(camera_.getOffsetFromPosition() * MOUSE_UZ_SENSITIVITY);
        } else if (distance < camera_.getZFar()) {// unzoom
            camera_.setOffsetFromPosition(camera_.getOffsetFromPosition() * MOUSE_DZ_SENSITIVITY);
        }
    }
#endif

    // Translation of the camera on its axis
    if (keyStates_['u'] == KEY_STATE::DOWN) {
        camera_.translate((camera_.getForward()*-1) * KEY_T_SENSITIVITY);
    }
    if (keyStates_['j'] == KEY_STATE::DOWN) {
        camera_.translate(camera_.getForward() * KEY_T_SENSITIVITY);
    }
    if (keyStates_['h'] == KEY_STATE::DOWN) {
        camera_.translate(camera_.getRight() * KEY_T_SENSITIVITY);
    }
    if (keyStates_['k'] == KEY_STATE::DOWN) {
        camera_.translate((camera_.getRight()*-1) * KEY_T_SENSITIVITY);
    }

    camera_.update();

    clearInputs();
}

void GLViewer::loadTexture() {
    /*   glEnable(GL_TEXTURE_2D);
       for (int i = 0; i < mesh_tex.size(); i++) {
           mesh_tex[i].texture.indice_gl = i;
           glGenTextures(1, &mesh_tex[i].texture.indice_gl);
           glBindTexture(GL_TEXTURE_2D, mesh_tex[i].texture.indice_gl);
           if (mesh_tex[i].texture.data.getChannels() == 3) {
               glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB
                   , mesh_tex[i].texture.data.getWidth()
                   , mesh_tex[i].texture.data.getHeight()
                   , 0, GL_BGR, GL_UNSIGNED_BYTE
                   , mesh_tex[i].texture.data.getPtr<sl::uchar1>());
           } else {
               glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA
                   , mesh_tex[i].texture.data.getWidth()
                   , mesh_tex[i].texture.data.getHeight()
                   , 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE
                   , mesh_tex[i].texture.data.getPtr<sl::uchar1>());
           }
           glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
           glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
       }*/
}

void GLViewer::draw() {
    //printf("GLViewer::draw()\n");
    const Eigen::Affine3f vpMatrix = camera_.getViewProjectionMatrix();

    // Simple 3D shader for simple 3D objects
    glUseProgram(shader_.getProgramId());
    // Axis
    //glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, vpMatrix.transpose().data());
    glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, vpMatrix.data());
    //glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, vpMatrix.matrix().transpose().data());

    glUniform3fv(shColorLoc_, 1, camRepere.clr.data());
    camRepere.draw();

    glUniform3fv(shColorLoc_, 1, grill.clr.data());
    grill.draw();

    glUseProgram(0);

    glUseProgram(shader_people.getProgramId());
    //glUniformMatrix4fv(shMVPMatrixLoc_people, 1, GL_FALSE, vpMatrix.transpose().data());
    glUniformMatrix4fv(shMVPMatrixLoc_people, 1, GL_FALSE, vpMatrix.data());
    //glUniformMatrix4fv(shMVPMatrixLoc_people, 1, GL_FALSE, vpMatrix.matrix().transpose().data());
    peopleObj.updateMesh();
    peopleObj.draw();

    glUseProgram(0);

    glUseProgram(shader_pc.getProgramId());
    //glUniformMatrix4fv(shMVPMatrixLoc_pc, 1, GL_FALSE, vpMatrix.transpose().data());
    glUniformMatrix4fv(shMVPMatrixLoc_pc, 1, GL_FALSE, vpMatrix.data());
    //glUniformMatrix4fv(shMVPMatrixLoc_pc, 1, GL_FALSE, vpMatrix.matrix().transpose().data());
    pointcloudObj.updateMesh();
    pointcloudObj.draw();
    glUseProgram(0);
}

void GLViewer::clearInputs() {
    mouseMotion_[0] = mouseMotion_[1] = 0;
    mouseWheelPosition_ = 0;
    for (unsigned int i = 0; i < 256; ++i)
        if (keyStates_[i] != KEY_STATE::DOWN)
            keyStates_[i] = KEY_STATE::FREE;
}

void GLViewer::drawCallback() {
    //printf("drawCallbak()\n");
    currentInstance_->render();
}

void GLViewer::mouseButtonCallback(int button, int state, int x, int y) {
    //printf("mouseButtonCallback\n");
    if (button < 5) {
        if (button < 3) {
            currentInstance_->mouseButton_[button] = state == GLUT_DOWN;
        } else {
            currentInstance_->mouseWheelPosition_ += button == MOUSE_BUTTON::WHEEL_UP ? 1 : -1;
        }
        currentInstance_->mouseCurrentPosition_[0] = x;
        currentInstance_->mouseCurrentPosition_[1] = y;
        currentInstance_->previousMouseMotion_[0] = x;
        currentInstance_->previousMouseMotion_[1] = y;
    }
}

void GLViewer::mouseMotionCallback(int x, int y) {
    //printf("mouseMotionCallback\n");
    currentInstance_->mouseMotion_[0] = x - currentInstance_->previousMouseMotion_[0];
    currentInstance_->mouseMotion_[1] = y - currentInstance_->previousMouseMotion_[1];
    currentInstance_->previousMouseMotion_[0] = x;
    currentInstance_->previousMouseMotion_[1] = y;
    glutPostRedisplay();
}

void GLViewer::reshapeCallback(int width, int height) {
    //printf("reshapeCallback\n");
    glViewport(0, 0, width, height);
    float hfov = currentInstance_->camera_.getHorizontalFOV();
    currentInstance_->camera_.setProjection(hfov, hfov * (float) height / (float) width, currentInstance_->camera_.getZNear(), currentInstance_->camera_.getZFar());
}

void GLViewer::keyPressedCallback(unsigned char c, int x, int y) {
    //printf("keyPressedCallback\n");
    currentInstance_->keyStates_[c] = KEY_STATE::DOWN;
    glutPostRedisplay();
}

void GLViewer::keyReleasedCallback(unsigned char c, int x, int y) {
    //printf("keyReleasedCallback\n");
    currentInstance_->keyStates_[c] = KEY_STATE::UP;
}

void GLViewer::idle() {
    //printf("glutPoseRedisplay\n");
    glutPostRedisplay();
}

Shader::Shader(GLchar* vs, GLchar* fs) {
    if (!compile(verterxId_, GL_VERTEX_SHADER, vs)) {
        std::cout << "ERROR: while compiling vertex shader" << std::endl;
    }
    if (!compile(fragmentId_, GL_FRAGMENT_SHADER, fs)) {
        std::cout << "ERROR: while compiling fragment shader" << std::endl;
    }

    programId_ = glCreateProgram();

    glAttachShader(programId_, verterxId_);
    glAttachShader(programId_, fragmentId_);

    glBindAttribLocation(programId_, ATTRIB_VERTICES_POS, "in_vertex");
    glBindAttribLocation(programId_, ATTRIB_COLOR_POS, "in_texCoord");

    glLinkProgram(programId_);

    GLint errorlk(0);
    glGetProgramiv(programId_, GL_LINK_STATUS, &errorlk);
    if (errorlk != GL_TRUE) {
        std::cout << "ERROR: while linking Shader :" << std::endl;
        GLint errorSize(0);
        glGetProgramiv(programId_, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(programId_, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteProgram(programId_);
    }
    //printf("Created Shader\n");
}

Shader::~Shader() {
    if (verterxId_ != 0)
        glDeleteShader(verterxId_);
    if (fragmentId_ != 0)
        glDeleteShader(fragmentId_);
    if (programId_ != 0)
        glDeleteShader(programId_);
}

GLuint Shader::getProgramId() {
    return programId_;
}

bool Shader::compile(GLuint &shaderId, GLenum type, GLchar* src) {
    //printf("GLShader::compile()\n");
    shaderId = glCreateShader(type);
    if (shaderId == 0) {
        std::cout << "ERROR: shader type (" << type << ") does not exist" << std::endl;
        return false;
    }
    glShaderSource(shaderId, 1, (const char**) &src, 0);
    glCompileShader(shaderId);

    GLint errorCp(0);
    glGetShaderiv(shaderId, GL_COMPILE_STATUS, &errorCp);
    if (errorCp != GL_TRUE) {
        std::cout << "ERROR: while compiling Shader :" << std::endl;
        GLint errorSize(0);
        glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(shaderId, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteShader(shaderId);
        return false;
    }
    return true;
}


const Eigen::Vector3f CameraGL::ORIGINAL_FORWARD = Eigen::Vector3f(0, 0, 1);
const Eigen::Vector3f CameraGL::ORIGINAL_UP = Eigen::Vector3f(0, 1, 0);
const Eigen::Vector3f CameraGL::ORIGINAL_RIGHT = Eigen::Vector3f(1, 0, 0);

CameraGL::CameraGL(Eigen::Vector3f position, Eigen::Vector3f direction, Eigen::Vector3f vertical) {
    this->position_ = position;
    setDirection(direction, vertical);

    offset_ = Eigen::Vector3f(0, 0, 4);
    view_.setIdentity();
    updateView();
    //setProjection(60, 60, 0.01f, 100.f);
    //setProjection(160, 160, .01f, 100.f);
    setProjection(130, 130, .01f, 100.f);
    updateVPMatrix();
    //printf("CameraGL::CameraGL()\n");
}

CameraGL::~CameraGL() {

}

void CameraGL::update() {
    //printf("CameraGL::update()\n");
    if (vertical_.dot(up_) < 0)
        vertical_ = vertical_ * -1;
    updateView();
    updateVPMatrix();
}

void CameraGL::setProjection(float horizontalFOV, float verticalFOV, float znear, float zfar) {
    //printf("CameraGL::setProjection()\n");
    horizontalFieldOfView_ = horizontalFOV;
    verticalFieldOfView_ = verticalFOV;
    znear_ = znear;
    zfar_ = zfar;

    float fov_y = verticalFOV * M_PI / 180.f;
    float fov_x = horizontalFOV * M_PI / 180.f;

    projection_.setIdentity();
    projection_(0, 0) = 1.0f / tanf(fov_x * 0.5f);
    projection_(1, 1) = 1.0f / tanf(fov_y * 0.5f);
    projection_(2, 2) = -(zfar + znear) / (zfar - znear);
    projection_(3, 2) = -1;
    projection_(2, 3) = -(2.f * zfar * znear) / (zfar - znear);
    projection_(3, 3) = 0;

#if 0
    std::cout << projection_.matrix() << std::endl;

    glm::mat4 proj;
    proj = glm::perspective(glm::radians(120.f), 1.f, 0.01f, 100.f);
    std::cout << glm::to_string(proj) << std::endl;
#endif
}

const Eigen::Affine3f& CameraGL::getViewProjectionMatrix() const {
    return vpMatrix_;
}

float CameraGL::getHorizontalFOV() const {
    return horizontalFieldOfView_;
}

float CameraGL::getVerticalFOV() const {
    return verticalFieldOfView_;
}

void CameraGL::setOffsetFromPosition(const Eigen::Vector3f& o) {
    offset_ = o;
}

const Eigen::Vector3f& CameraGL::getOffsetFromPosition() const {
    return offset_;
}

void CameraGL::setDirection(const Eigen::Vector3f& direction, const Eigen::Vector3f& vertical) {
    //printf("CameraGL::setDirection()\n");
    Eigen::Vector3f dirNormalized = direction.normalized();
    //this->rotation_ = orientation_mat(ORIGINAL_FORWARD, dirNormalized * -1.f);
    Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(ORIGINAL_FORWARD, dirNormalized * -1);
    updateVectors();
    this->vertical_ = vertical;
    Eigen::Vector3f aux(vertical_);
    if (aux.dot(up_) < 0)
    {
	    // Create proper axis angle representation
	    Eigen::AngleAxisf axis_angle(M_PI, ORIGINAL_FORWARD.normalized());
	    rotate(Eigen::Quaternionf(axis_angle));
    }
}

void CameraGL::translate(const Eigen::Vector3f& t) {
    position_ = position_ + t;
}

void CameraGL::setPosition(const Eigen::Vector3f& p) {
    position_ = p;
}

void CameraGL::rotate(const Eigen::Quaternionf& rot) {
    rotation_ = (rot * rotation_).normalized();
    updateVectors();
}

void CameraGL::rotate(const Eigen::Matrix3f& m) {
    this->rotate(Eigen::Quaternionf(m));
}

void CameraGL::setRotation(const Eigen::Quaternionf& rot) {
    rotation_ = rot.normalized();
    updateVectors();
}

void CameraGL::setRotation(const Eigen::Matrix3f& m) {
    this->setRotation(Eigen::Quaternionf(m));
}

const Eigen::Vector3f& CameraGL::getPosition() const {
    return position_;
}

const Eigen::Vector3f& CameraGL::getForward() const {
    return forward_;
}

const Eigen::Vector3f& CameraGL::getRight() const {
    return right_;
}

const Eigen::Vector3f& CameraGL::getUp() const {
    return up_;
}

const Eigen::Vector3f& CameraGL::getVertical() const {
    return vertical_;
}

float CameraGL::getZNear() const {
    return znear_;
}

float CameraGL::getZFar() const {
    return zfar_;
}

void CameraGL::updateVectors() {
    //printf("CameraGL::updateViews()\n");
    forward_ = rotation_._transformVector(ORIGINAL_FORWARD);
    up_ = rotation_._transformVector(ORIGINAL_UP);
    right_ = rotation_._transformVector(Eigen::Vector3f(ORIGINAL_RIGHT * -1));
}

void CameraGL::updateView() {
    //printf("CameraGL::updateView()\n");
    //Eigen::Affine3f transformation(Eigen::Affine3f::Identity());
    // Extract rotation from class rotation_ object
    //Eigen::Quaternionf quat(rotation_);
    //quat.normalize();
    //Eigen::Matrix3f associated_rotation = quat.toRotationMatrix();
    // Apply rotation
#if 0
    std::cout << "rot: " << std::endl;
    std::cout << associated_rotation << std::endl;
#endif
    //transformation.rotate(associated_rotation);
    Eigen::Affine3f transformation(rotation_);
    //transformation.rotate(rotation_);
    // Rotate offset by rotation quat and add position translation
    Eigen::Vector3f transl = rotation_._transformVector(offset_) + position_;
#if 0
    std::cout << transl << std::endl;
    Eigen::Matrix3f ass_rot = rotation_.normalized().toRotationMatrix();
    Eigen::Vector3f new_transl = ass_rot * offset_ + position_;
    std::cout << new_transl << std::endl;
    std::cout << " " << std::endl;
#endif
#if 0
    std::cout << "transl: " << std::endl;
    std::cout << transl << std::endl;
#endif
    transformation.pretranslate(transl);
#if 0
    std::cout << "transf: " << std::endl;
    std::cout << transformation.matrix() << std::endl;
#endif

    // update view_
    view_ = transformation.inverse();
}

void CameraGL::updateVPMatrix() {
    //printf("CameraGL::updateVPMatrix()\n");
    vpMatrix_ = projection_ * view_;
    //vpMatrix_ = view_ * projection_;
}
