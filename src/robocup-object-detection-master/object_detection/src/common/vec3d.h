#include <nav_msgs/Odometry.h>

// Basic 3d vector and 3x3 matrix operations
struct vec3d
{
    constexpr vec3d(double x = 0., double y = 0., double z = 0.) : x(x), y(y), z(z)
    {
    }
    explicit constexpr vec3d(const geometry_msgs::Point &p) : x(p.x), y(p.y), z(p.z)
    {
    }
    explicit constexpr vec3d(float p[3]) : x(p[0]), y(p[1]), z(p[2])
    {
    }

    constexpr vec3d operator+(const vec3d &v) const
    {
        return vec3d{x + v.x, y + v.y, z + v.z};
    }
    void operator+=(const vec3d &v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }
    constexpr vec3d operator+() const
    {
        return *this;
    }
    constexpr vec3d operator-(const vec3d &v) const
    {
        return vec3d{x - v.x, y - v.y, z - v.z};
    }
    constexpr vec3d operator-() const
    {
        return vec3d{-x, -y, -z};
    }

    constexpr vec3d operator*(double a) const
    {
        return vec3d{a * x, a * y, a * z};
    }

    constexpr double dot(const vec3d &v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    explicit operator geometry_msgs::Point() const
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }
    constexpr double &operator[](const size_t i)
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;

        default:
            assert(0);
            return x; // just to satisfy the compiler
        }
    }
    double x, y, z;
};

struct matrix3d
{
public:
    constexpr matrix3d(const vec3d &x, const vec3d &y, const vec3d &z) : x(x), y(y), z(z)
    {
    }
    explicit constexpr matrix3d(const geometry_msgs::Quaternion &Q)
    {
        matrix3d &rot = *this;
        // make aliases so copying code from
        // https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
        // is easier
        const double &q0 = Q.w, &q1 = Q.x, &q2 = Q.y, &q3 = Q.z;
        double &r00 = rot[0][0], &r01 = rot[0][1], &r02 = rot[0][2];
        double &r10 = rot[1][0], &r11 = rot[1][1], &r12 = rot[1][2];
        double &r20 = rot[2][0], &r21 = rot[2][1], &r22 = rot[2][2];

        r00 = 2 * (q0 * q0 + q1 * q1) - 1;
        r01 = 2 * (q1 * q2 - q0 * q3);
        r02 = 2 * (q1 * q3 + q0 * q2);

        r10 = 2 * (q1 * q2 + q0 * q3);
        r11 = 2 * (q0 * q0 + q2 * q2) - 1;
        r12 = 2 * (q2 * q3 - q0 * q1);

        r20 = 2 * (q1 * q3 - q0 * q2);
        r21 = 2 * (q2 * q3 + q0 * q1);
        r22 = 2 * (q0 * q0 + q3 * q3) - 1;
    }
    constexpr vec3d dot(const vec3d &v) const
    {
        return vec3d{x.dot(v), y.dot(v), z.dot(v)};
    }
    constexpr vec3d &operator[](const size_t i)
    {
        assert(i >= 0 && i < 3);
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;

        default:
            return x; // just to satisfy the compiler
        }
    }

private:
    vec3d x, y, z;
};