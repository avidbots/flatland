#ifndef FLATLAND_BOX2D_COMPAT_H
#define FLATLAND_BOX2D_COMPAT_H

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace flatland_box2d_v3
{
#include <box2d/box2d.h>
}

#include <array>
#include <map>
#include <memory>
#include <utility>
#include <vector>

struct b2Vec2
{
  float x = 0.0f;
  float y = 0.0f;

  b2Vec2() = default;
  b2Vec2(float x, float y) : x(x), y(y) {}
  b2Vec2(flatland_box2d_v3::b2Vec2 value) : x(value.x), y(value.y) {}
  operator flatland_box2d_v3::b2Vec2() const { return {x, y}; }
  void Set(float new_x, float new_y) { x = new_x; y = new_y; }
  b2Vec2 operator+(b2Vec2 other) const { return {x + other.x, y + other.y}; }
  b2Vec2 operator-(b2Vec2 other) const { return {x - other.x, y - other.y}; }
  b2Vec2 operator*(float factor) const { return {x * factor, y * factor}; }
  b2Vec2 & operator*=(float factor) { x *= factor; y *= factor; return *this; }
};

inline b2Vec2 operator*(float factor, b2Vec2 value) { return value * factor; }
using uint32 = std::uint32_t;

using b2BodyType = flatland_box2d_v3::b2BodyType;
constexpr b2BodyType b2_staticBody = flatland_box2d_v3::b2_staticBody;
constexpr b2BodyType b2_kinematicBody = flatland_box2d_v3::b2_kinematicBody;
constexpr b2BodyType b2_dynamicBody = flatland_box2d_v3::b2_dynamicBody;
constexpr int b2_maxPolygonVertices = B2_MAX_POLYGON_VERTICES;
using b2Filter = flatland_box2d_v3::b2Filter;
using b2Manifold = flatland_box2d_v3::b2Manifold;

struct b2Transform
{
  b2Vec2 p;
  struct { float c = 1.0f; float s = 0.0f; } q;
};

inline b2Vec2 b2MulT(const b2Transform & transform, b2Vec2 point)
{
  point = point - transform.p;
  return {transform.q.c * point.x + transform.q.s * point.y,
          -transform.q.s * point.x + transform.q.c * point.y};
}

class b2Shape
{
public:
  enum Type { e_circle, e_edge, e_polygon, e_chain };
  virtual ~b2Shape() = default;
  virtual Type GetType() const = 0;
};

class b2CircleShape : public b2Shape
{
public:
  b2Vec2 m_p;
  float m_radius = 0.0f;
  Type GetType() const override { return e_circle; }
};

class b2EdgeShape : public b2Shape
{
public:
  b2Vec2 m_vertex1, m_vertex2;
  void Set(b2Vec2 start, b2Vec2 end) { m_vertex1 = start; m_vertex2 = end; }
  Type GetType() const override { return e_edge; }
};

class b2PolygonShape : public b2Shape
{
public:
  b2Vec2 m_vertices[b2_maxPolygonVertices] = {};
  int m_count = 0;
  void Set(const b2Vec2 * vertices, int count);
  void SetAsBox(float half_width, float half_height);
  Type GetType() const override { return e_polygon; }
};

class b2ChainShape : public b2Shape
{
public:
  std::vector<b2Vec2> vertices;
  void CreateLoop(const b2Vec2 * points, int count) { vertices.assign(points, points + count); }
  void CreateChain(const b2Vec2 * points, int count) { vertices.assign(points, points + count); }
  Type GetType() const override { return e_chain; }
};

struct b2FixtureDef
{
  const b2Shape * shape = nullptr;
  float density = 0.0f;
  float friction = 0.2f;
  float restitution = 0.0f;
  bool isSensor = false;
  b2Filter filter = flatland_box2d_v3::b2DefaultFilter();
};

struct b2BodyDef
{
  b2BodyType type = b2_staticBody;
  b2Vec2 position;
  float angle = 0.0f;
  float linearDamping = 0.0f;
  float angularDamping = 0.0f;
};

class b2World;
class b2Body;
class b2Fixture
{
public:
  b2Fixture(b2Body * body, flatland_box2d_v3::b2ShapeId id, std::unique_ptr<b2Shape> shape);
  b2Body * GetBody() const { return body_; }
  b2Fixture * GetNext() const { return next_; }
  b2Shape * GetShape() const { return shape_.get(); }
  b2Shape::Type GetType() const { return shape_->GetType(); }
  b2Filter GetFilterData() const;
  bool IsSensor() const;
  float GetDensity() const;
  float GetFriction() const;
  float GetRestitution() const;
  void SetSensor(bool sensor);

  b2Body * body_;
  flatland_box2d_v3::b2ShapeId id_;
  std::unique_ptr<b2Shape> shape_;
  b2Fixture * next_ = nullptr;
};

class b2Body
{
public:
  b2Body(b2World * world, flatland_box2d_v3::b2BodyId id);
  b2World * GetWorld() const { return world_; }
  b2BodyType GetType() const;
  void SetType(b2BodyType type);
  void SetUserData(void * data);
  void * GetUserData() const;
  b2Fixture * CreateFixture(const b2FixtureDef * definition);
  b2Fixture * CreateFixture(const b2Shape * shape, float density);
  b2Fixture * GetFixtureList() const { return fixtures_; }
  b2Vec2 GetPosition() const;
  float GetAngle() const;
  b2Transform GetTransform() const;
  void SetTransform(b2Vec2 position, float angle);
  b2Vec2 GetLocalPoint(b2Vec2 point) const;
  b2Vec2 GetWorldPoint(b2Vec2 point) const;
  b2Vec2 GetWorldVector(b2Vec2 vector) const;
  b2Vec2 GetWorldCenter() const;
  b2Vec2 GetLinearVelocity() const;
  float GetAngularVelocity() const;
  b2Vec2 GetLinearVelocityFromLocalPoint(b2Vec2 point) const;
  float GetLinearDamping() const;
  float GetAngularDamping() const;
  void SetLinearVelocity(b2Vec2 velocity);
  void SetAngularVelocity(float velocity);
  void SetAwake(bool awake);
  void Dump() const;

  b2World * world_;
  flatland_box2d_v3::b2BodyId id_;
  b2Fixture * fixtures_ = nullptr;
  std::vector<std::unique_ptr<b2Fixture>> owned_fixtures_;
};

enum b2JointType { e_unknownJoint, e_revoluteJoint, e_weldJoint, e_distanceJoint,
                   e_pulleyJoint, e_mouseJoint };

struct b2JointDef
{
  virtual ~b2JointDef() = default;
  b2Body * bodyA = nullptr;
  b2Body * bodyB = nullptr;
  bool collideConnected = false;
};

struct b2RevoluteJointDef : b2JointDef
{
  b2Vec2 localAnchorA, localAnchorB;
  bool enableLimit = false;
  float lowerAngle = 0.0f, upperAngle = 0.0f;
};

struct b2WeldJointDef : b2JointDef
{
  b2Vec2 localAnchorA, localAnchorB;
  float referenceAngle = 0.0f, frequencyHz = 0.0f, dampingRatio = 0.0f;
};

class b2Joint
{
public:
  b2Joint(b2Body * body_a, b2Body * body_b, flatland_box2d_v3::b2JointId id);
  virtual ~b2Joint() = default;
  virtual b2JointType GetType() const = 0;
  b2Body * GetBodyA() const { return body_a_; }
  b2Body * GetBodyB() const { return body_b_; }
  b2Vec2 GetAnchorA() const;
  b2Vec2 GetAnchorB() const;
  bool GetCollideConnected() const;
  void SetUserData(void * data);
  void * GetUserData() const;
  void Dump() const;
  b2Body * body_a_;
  b2Body * body_b_;
  flatland_box2d_v3::b2JointId id_;
};

class b2RevoluteJoint : public b2Joint
{
public:
  using b2Joint::b2Joint;
  b2JointType GetType() const override { return e_revoluteJoint; }
  bool IsLimitEnabled() const;
  float GetLowerLimit() const;
  float GetUpperLimit() const;
  void EnableLimit(bool enabled);
  void SetLimits(float lower, float upper);
  float configured_lower_limit_ = 0.0f;
  float configured_upper_limit_ = 0.0f;
};

class b2WeldJoint : public b2Joint
{
public:
  using b2Joint::b2Joint;
  b2JointType GetType() const override { return e_weldJoint; }
  float GetReferenceAngle() const;
  float GetFrequency() const;
  float GetDampingRatio() const;
};

struct b2ContactImpulse
{
  float normalImpulses[2] = {};
  float tangentImpulses[2] = {};
  int count = 0;
};

struct b2WorldManifold
{
  b2Vec2 normal;
  b2Vec2 points[2];
};

class b2Contact
{
public:
  b2Contact(b2Fixture * fixture_a, b2Fixture * fixture_b);
  b2Fixture * GetFixtureA() const { return fixture_a_; }
  b2Fixture * GetFixtureB() const { return fixture_b_; }
  b2Manifold * GetManifold() { return &manifold_; }
  void GetWorldManifold(b2WorldManifold * output) const;
  b2Fixture * fixture_a_;
  b2Fixture * fixture_b_;
  b2Manifold manifold_ = {};
};

class b2ContactListener
{
public:
  virtual ~b2ContactListener() = default;
  virtual void BeginContact(b2Contact *) {}
  virtual void EndContact(b2Contact *) {}
  virtual void PostSolve(b2Contact *, const b2ContactImpulse *) {}
};

class b2RayCastCallback
{
public:
  virtual ~b2RayCastCallback() = default;
  virtual float ReportFixture(b2Fixture *, const b2Vec2 &, const b2Vec2 &, float) = 0;
};

class b2World
{
public:
  explicit b2World(b2Vec2 gravity);
  ~b2World();
  b2Body * CreateBody(const b2BodyDef * definition);
  void DestroyBody(b2Body * body);
  b2Joint * CreateJoint(const b2JointDef * definition);
  void DestroyJoint(b2Joint * joint);
  void SetContactListener(b2ContactListener * listener) { listener_ = listener; }
  void Step(float time_step, int velocity_iterations, int position_iterations);
  void RayCast(b2RayCastCallback * callback, b2Vec2 start, b2Vec2 end) const;
  flatland_box2d_v3::b2WorldId id_;

private:
  friend class b2Fixture;
  using ContactKey = std::pair<b2Fixture *, b2Fixture *>;
  static ContactKey Key(b2Fixture * first, b2Fixture * second);
  b2Fixture * FindFixture(flatland_box2d_v3::b2ShapeId id) const;
  void EndContactsFor(b2Body * body);
  std::vector<std::unique_ptr<b2Body>> bodies_;
  std::vector<std::unique_ptr<b2Joint>> joints_;
  std::map<ContactKey, std::unique_ptr<b2Contact>> contacts_;
  b2ContactListener * listener_ = nullptr;
};

void b2Log(const char * format, ...);

#endif