#include <le3.h>
#include <fmt/core.h>
using namespace le3;
using fmt::print, fmt::format;

#include <fdml/fdml.h>
#include "cgal_stuff.h"

class TriangleObject : public LE3DrawableObject {
public:
    fdml::Triangle3<double> m_t;
    glm::vec3 m_color;
    int m_splits = 4;

    TriangleObject(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 color) : LE3DrawableObject(nullptr) {
        m_t.a.data[0] = a.x; m_t.a.data[1] = a.y; m_t.a.data[2] = a.z;
        m_t.b.data[0] = b.x; m_t.b.data[1] = b.y; m_t.b.data[2] = b.z;
        m_t.c.data[0] = c.x; m_t.c.data[1] = c.y; m_t.c.data[2] = c.z;
        m_color = color;
    }

    glm::vec3 getVertex(int idx) const{
        fdml::Point3<double> v;
        switch(idx % 3) {
        case 0: v = m_t.a; break;
        case 1: v = m_t.b; break;
        case 2: v = m_t.c; break;
        }
        return glm::vec3(v.data[0], v.data[1], v.data[2]);
    }

    CGAL_Point_3 getVertexCGAL(int idx) const {
        glm::vec3 v = getVertex(idx);
        return CGAL_Point_3(v.x, v.y, v.z);
    }

    CGAL_Triangle_3 toCGAL() const {
        return CGAL_Triangle_3(getVertexCGAL(0), getVertexCGAL(1), getVertexCGAL(2));
    }

    void draw(LE3ShaderPtr shaderOverride) {
        // Draw the triangle itself
        for (int i = 0; i < 4; i++) 
            LE3GetVisualDebug().drawDebugLine(getVertex(i), getVertex(i+1), m_color);

        // Also draw fan to make this more clear
        for (int i = 0; i < 3; i++) {
            glm::vec3 root = getVertex(i);
            glm::vec3 v1 = getVertex(i+1);
            glm::vec3 v2 = getVertex(i+2);

            for (int j = 0; j < m_splits; j++) {
                float alpha = (float)j / (float)(m_splits - 1);
                glm::vec3 target = v1 + alpha * (v2 - v1);
                LE3GetVisualDebug().drawDebugLine(root, target, m_color);
            }
        }
    }
};


class PredicateVisualization : public LE3SimpleDemo {
public:
    void init() {
        LE3SimpleDemo::init();

        // Some Visual stuff
        LE3GetAssetManager().getMaterial("M_default")->specularIntensity = 0.f;
        m_scene.addAmbientLight("ambientLight");
        m_scene.getObject<LE3AmbientLight>("ambientLight")->setIntensity(0.3);
        m_scene.addBox("floor", "M_default", glm::vec3(0.f, -1.1f, 0.f), glm::vec3(50.f, 0.1f, 50.f));

        m_t1 = std::make_shared<TriangleObject>(
            glm::vec3(0, 1, 0), 
            glm::vec3(1, 1, 0), 
            glm::vec3(1, 1, 1),
            glm::vec3(1, 0, 0)); //color
        m_t2 = std::make_shared<TriangleObject>(
            glm::vec3(0, 0.5, 0), 
            glm::vec3(1, 1, 0), 
            glm::vec3(1, 1.5, 1),
            glm::vec3(0, 1, 0)); //color

        m_scene.addCustomObject("t1", m_t1);
        m_scene.addCustomObject("t2", m_t2);
    }

    void update(float deltaTime) {
        LE3SimpleDemo::update(deltaTime);
    }

    void renderDebug() {
        CGAL_Triangle_3 t1 = m_t1->toCGAL();
        CGAL_Triangle_3 t2 = m_t2->toCGAL();
        auto res = CGAL::intersection(t1, t2);
        if (res) {
            if (std::holds_alternative<CGAL_Segment_3>(res.value())) {
                CGAL_Segment_3 s = std::get<CGAL_Segment_3>(res.value());
                LE3GetVisualDebug().drawDebugLine(cgal2glm(s.source()), cgal2glm(s.target()), glm::vec3(1, 0, 1));
            }
        }
    }

protected:
    std::shared_ptr<TriangleObject> m_t1, m_t2;
};

int main() {
    fdml::Random::seed(0);
    LE3Application app(std::make_unique<PredicateVisualization>());
    app.run();
    return 0;
}