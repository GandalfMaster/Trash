#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
    std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const& points)
    {
        size_t size = points.size();
        std::vector<Vector2D> res;
        for (int i = 0; i < size - 1; i++) {
            Vector2D temp;
            temp.x = (1 - t) * points[i].x + t * points[i + 1].x;
            temp.y = (1 - t) * points[i].y + t * points[i + 1].y;

            res.push_back(temp);
        }

        return res;
    }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
    std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const& points, double t) const
    {
        size_t size = points.size();
        std::vector<Vector3D> res;
        if (size < 2)return res;
        for (int i = 0; i < size - 1; i++) {
            Vector3D temp;
            temp.x = (1 - t) * points[i].x + t * points[i + 1].x;
            temp.y = (1 - t) * points[i].y + t * points[i + 1].y;
            temp.z = (1 - t) * points[i].z + t * points[i + 1].z;

            res.push_back(temp);
        }

        return res;
    }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
    Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
    {   
        std::vector<Vector3D> temp = points;
        while (temp.size() > 1) {
            temp = evaluateStep(temp, t);
        }
        return temp.empty() ? Vector3D() : temp.back();
    }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
    Vector3D BezierPatch::evaluate(double u, double v) const 
    {  
        std::vector<Vector3D> temp;
        if (!controlPoints.size())return Vector3D();
        for (int i = 0; i < controlPoints.size(); i++) {
            temp.push_back(evaluate1D(controlPoints[i], u));
        }

        return evaluate1D(temp, v);
    }

    Vector3D Vertex::normal( void ) const
    {
        Vector3D res;
        HalfedgeCIter h = halfedge();

        HalfedgeCIter ptr = h;

        do {
            Vector3D p0 = h->vertex()->position;
            Vector3D p1 = h->next()->vertex()->position;
            Vector3D p2 = h->next()->next()->vertex()->position;

            Vector3D c = cross(p1 - p0, p2 - p0);
            //Vector3D c = cross(p0 - p1, p0 - p2);
            //c *= -1;
            double area = c.norm() / 2.0;
            res += c * area;

            h = h->twin()->next();
        } while (h != ptr);

        return res.unit();
    }

    EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
    {
        HalfedgeIter h1 = e0->halfedge();
        HalfedgeIter h2 = h1->twin();

        if (h1->isBoundary() || h2->isBoundary()) {
            return e0;
        }
        VertexIter v1 = h1->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h1->next()->next()->vertex();
        VertexIter v4 = h2->next()->next()->vertex();
        

        FaceIter f1 = h1->face();
        FaceIter f2 = h2->face();

        HalfedgeIter h1_next = h1->next();
        HalfedgeIter h2_next = h2->next();
        HalfedgeIter h1_prev = h1_next->next();
        HalfedgeIter h2_prev = h2_next->next();

        EdgeIter e1 = h1_next->edge();
        EdgeIter e2 = h2_next->edge();
        EdgeIter e3 = h1_prev->edge();
        EdgeIter e4 = h2_prev->edge();


        h1->setNeighbors(h1_prev, h2, v4, e0, f1);
        h2->setNeighbors(h2_prev, h1, v3, e0, f2);

        h1_next->setNeighbors(h2, h1_next->twin(), v2, h1_next->edge(), f2);
        h2_next->setNeighbors(h1, h2_next->twin(), v1, h2_next->edge(), f1);

        h1_prev->setNeighbors(h2_next, h1_prev->twin(), v3, h1_prev->edge(), f1);
        h2_prev->setNeighbors(h1_next, h2_prev->twin(), v4, h2_prev->edge(), f2);


        f1->halfedge() = h1;
        f2->halfedge() = h2;

        e0->halfedge() = h1;
        e1->halfedge() = h1_next;
        e2->halfedge() = h2_next;
        e3->halfedge() = h1_prev;
        e4->halfedge() = h2_prev;

        v1->halfedge() = h2_next;
        v2->halfedge() = h1_next;
        v3->halfedge() = h1_prev;
        v4->halfedge() = h2_prev;

        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e) {
        HalfedgeIter h1 = e->halfedge();
        HalfedgeIter h2 = h1->twin();

        if (h1->isBoundary() || h2->isBoundary()) {
            return VertexIter(); 
        }

        VertexIter v1 = h1->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h1->next()->next()->vertex();
        VertexIter v4 = h2->next()->next()->vertex();

        HalfedgeIter h1_next = h1->next();
        HalfedgeIter h2_next = h2->next();
        HalfedgeIter h1_prev = h1_next->next();
        HalfedgeIter h2_prev = h2_next->next();


        Vector3D midpoint = (v1->position + v2->position) * 0.5;
        VertexIter newVertexIter = newVertex();
        newVertexIter->position = midpoint;
        newVertexIter->isNew = true;

        HalfedgeIter newH1 = h1;
        HalfedgeIter newH2 = h2;
        HalfedgeIter newH3 = newHalfedge();
        HalfedgeIter newH4 = newHalfedge();
        HalfedgeIter newH5 = newHalfedge();
        HalfedgeIter newH6 = newHalfedge();
        HalfedgeIter newH7 = newHalfedge();
        HalfedgeIter newH8 = newHalfedge();
        
        EdgeIter newE1 = h1->edge();
        EdgeIter newE2 = newEdge();
        EdgeIter newE3 = newEdge();
        EdgeIter newE4 = newEdge();

        EdgeIter E1 = h1_next->edge();
        EdgeIter E2 = h2_next->edge();
        EdgeIter E3 = h1_prev->edge();
        EdgeIter E4 = h2_prev->edge();


        FaceIter newF1 = h1->face();
        FaceIter newF2 = h2->face();
        FaceIter newF3 = newFace();
        FaceIter newF4 = newFace();

        newH1->setNeighbors(newH5, newH2, v1, newE1, newF1);
        newH2->setNeighbors(h2_next, newH1, newVertexIter, newE1, newF2);
        newH3->setNeighbors(h1_next, newH4, newVertexIter, newE2, newF3);
        newH4->setNeighbors(newH8, newH3, v2, newE2, newF4);
        newH5->setNeighbors(h1_prev, newH6, newVertexIter, newE3, newF1);
        newH6->setNeighbors(newH3, newH5, v3, newE3, newF3);
        newH7->setNeighbors(newH2, newH8, v4, newE4, newF2);
        newH8->setNeighbors(h2_prev, newH7, newVertexIter, newE4, newF4);

        h1_next->setNeighbors(newH6, h1_next->twin(), v2, h1_next->edge(), newF3);
        h2_next->setNeighbors(newH7, h2_next->twin(), v1, h2_next->edge(), newF2);
        h1_prev->setNeighbors(newH1, h1_prev->twin(), v3, h1_prev->edge(), newF1);
        h2_prev->setNeighbors(newH4, h2_prev->twin(), v4, h2_prev->edge(), newF4);

        v1->halfedge() = newH1;
        v2->halfedge() = newH4;
        v3->halfedge() = newH6;
        v4->halfedge() = newH7;
        newVertexIter->halfedge() = newH2;

        newE1->halfedge() = newH1;
        //newE1->isNew = true;
        newE2->halfedge() = newH3;
        //newE2->isNew = true;
        newE3->halfedge() = newH5;
        newE3->isNew = true;
        newE4->halfedge() = newH7;
        newE4->isNew = true;

        E1->halfedge() = h1_next;
        E2->halfedge() = h2_next;
        E3->halfedge() = h1_prev;
        E4->halfedge() = h2_prev;

        newF1->halfedge() = newH1;
        newF2->halfedge() = newH2;
        newF3->halfedge() = newH3;
        newF4->halfedge() = newH4;


        return newVertexIter;
    }




    void MeshResampler::upsample(HalfedgeMesh& mesh) {
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->isNew = false;
        }
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            e->isNew = false;
        }

        std::vector<EdgeIter> edgesToSplit;
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
            HalfedgeIter h = e->halfedge();
            Vector3D A = h->vertex()->position;
            Vector3D B = h->twin()->vertex()->position;
            Vector3D C = h->next()->next()->vertex()->position;
            Vector3D D = h->twin()->next()->next()->vertex()->position;
            e->newPosition = 3.0f / 8.0f * (A + B) + 1.0f / 8.0f * (C + D);
            edgesToSplit.push_back(e);
        }
        for (EdgeIter e : edgesToSplit) {
            VertexIter newVertex = mesh.splitEdge(e);
            newVertex->position = e->newPosition;
        }

        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
            if (e->isNew) {
                VertexIter v1 = e->halfedge()->vertex();
                VertexIter v2 = e->halfedge()->twin()->vertex();

                if ((v1->isNew && !v2->isNew)||(!v1->isNew && v2->isNew)) {
                    mesh.flipEdge(e);
                }
            }
        }

        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
            if (!v->isNew) {
                size_t n = v->degree();
                double u = (n == 3) ? (3.0 / 16.0) : (3.0 / (8.0 * n));
                Vector3D neighborSum(0, 0, 0);

                HalfedgeIter h = v->halfedge();
                HalfedgeIter hStart = h;
                do {
                    neighborSum += h->twin()->vertex()->position;
                    h = h->twin()->next();
                } while (h != hStart);

                v->newPosition = (1.0 - n * u) * v->position + u * neighborSum;
            }
        }

        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
            if (!v->isNew) {
                v->position = v->newPosition;
            }
        }

        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            if (!v->isNew) {
                Vector3D newPos = Vector3D(0, 0, 0);
                HalfedgeIter h = v->halfedge();
                do {
                    newPos += h->twin()->vertex()->position;
                    h = h->twin()->next();
                } while (h != v->halfedge());
                size_t n = v->degree();
                double u = (n == 3) ? (3.0 / 16.0) : (3.0 / (8.0 * n));
                v->position = (1.0 - n * u) * v->position + u * newPos;
            }
        }
    }







}



