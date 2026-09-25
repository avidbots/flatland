#include <Box2D/Box2D.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <numbers>
#include <stdexcept>

namespace flatland
{

void b2PolygonShape::Set(const b2Vec2 * vertices, int count)
{
  if (count < 3 || count > b2_maxPolygonVertices) {
    throw std::invalid_argument("Invalid Box2D polygon vertex count");
  }
  ::b2Vec2 points[b2_maxPolygonVertices];
  for (int index = 0; index < count; ++index) points[index] = vertices[index];
  auto hull = ::b2ComputeHull(points, count);
  if (hull.count < 3) throw std::invalid_argument("Box2D polygon is degenerate");
  m_count = hull.count;
  int start = 0;
  for (int index = 1; index < m_count; ++index) {
    if (hull.points[index].x > hull.points[start].x ||
        (hull.points[index].x == hull.points[start].x &&
         hull.points[index].y < hull.points[start].y)) {
      start = index;
    }
  }
  for (int index = 0; index < m_count; ++index) {
    m_vertices[index] = hull.points[(start + index) % m_count];
  }
}

void b2PolygonShape::SetAsBox(float half_width, float half_height)
{
  b2Vec2 corners[] = {{-half_width, -half_height}, {half_width, -half_height},
                       {half_width, half_height}, {-half_width, half_height}};
  m_count = 4;
  std::copy_n(corners, m_count, m_vertices);
}

b2Fixture::b2Fixture(b2Body * body, ::b2ShapeId id, std::unique_ptr<b2Shape> shape)
: body_(body), id_(id), shape_(std::move(shape)) {}

b2Filter b2Fixture::GetFilterData() const { return ::b2Shape_GetFilter(id_); }
bool b2Fixture::IsSensor() const { return ::b2Shape_IsSensor(id_); }
float b2Fixture::GetDensity() const { return ::b2Shape_GetDensity(id_); }
float b2Fixture::GetFriction() const { return ::b2Shape_GetFriction(id_); }
float b2Fixture::GetRestitution() const { return ::b2Shape_GetRestitution(id_); }
void b2Fixture::SetSensor(bool sensor)
{
  if (sensor == IsSensor()) return;
  if (shape_->GetType() == b2Shape::e_chain) {
    throw std::logic_error("Box2D 3.1 chains cannot be sensors");
  }
  body_->world_->EndContactsFor(body_, this);
  auto definition = ::b2DefaultShapeDef();
  definition.density = GetDensity();
  definition.material = ::b2Shape_GetSurfaceMaterial(id_);
  definition.filter = GetFilterData();
  definition.isSensor = sensor;
  definition.enableSensorEvents = true;
  definition.enableContactEvents = true;
  auto old_id = id_;
  switch (shape_->GetType()) {
    case b2Shape::e_circle: {
      auto circle = ::b2Shape_GetCircle(old_id);
      id_ = ::b2CreateCircleShape(body_->id_, &definition, &circle);
      break;
    }
    case b2Shape::e_edge: {
      auto segment = ::b2Shape_GetSegment(old_id);
      id_ = ::b2CreateSegmentShape(body_->id_, &definition, &segment);
      break;
    }
    case b2Shape::e_polygon: {
      auto polygon = ::b2Shape_GetPolygon(old_id);
      id_ = ::b2CreatePolygonShape(body_->id_, &definition, &polygon);
      break;
    }
    case b2Shape::e_chain:
      break;
  }
  if (!::b2Shape_IsValid(id_)) throw std::runtime_error("Failed to change sensor state");
  ::b2Shape_SetUserData(id_, this);
  ::b2DestroyShape(old_id, true);
}

b2Body::b2Body(b2World * world, ::b2BodyId id) : world_(world), id_(id) {}
b2BodyType b2Body::GetType() const { return ::b2Body_GetType(id_); }
void b2Body::SetType(b2BodyType type) { ::b2Body_SetType(id_, type); }
void b2Body::SetUserData(void * data) { ::b2Body_SetUserData(id_, data); }
void * b2Body::GetUserData() const { return ::b2Body_GetUserData(id_); }

b2Fixture * b2Body::CreateFixture(const b2Shape * shape, float density)
{
  b2FixtureDef definition;
  definition.shape = shape;
  definition.density = density;
  return CreateFixture(&definition);
}

b2Fixture * b2Body::CreateFixture(const b2FixtureDef * definition)
{
  if (definition == nullptr || definition->shape == nullptr) {
    throw std::invalid_argument("Box2D fixture requires geometry");
  }
  ::b2ShapeDef shape_def = ::b2DefaultShapeDef();
  shape_def.density = definition->density;
  shape_def.material.friction = definition->friction;
  shape_def.material.restitution = definition->restitution;
  shape_def.isSensor = definition->isSensor;
  shape_def.filter = definition->filter;
  shape_def.enableContactEvents = true;
  shape_def.enableSensorEvents = true;

  if (definition->shape->GetType() == b2Shape::e_chain) {
    const auto & chain = *static_cast<const b2ChainShape *>(definition->shape);
    std::vector<::b2Vec2> points;
    for (const auto & vertex : chain.vertices) points.push_back(vertex);
    auto chain_def = ::b2DefaultChainDef();
    chain_def.points = points.data();
    chain_def.count = points.size();
    chain_def.filter = definition->filter;
    chain_def.isLoop = chain.is_loop_;
    auto chain_id = ::b2CreateChain(id_, &chain_def);
    std::vector<::b2ShapeId> segments(::b2Chain_GetSegmentCount(chain_id));
    ::b2Chain_GetSegments(chain_id, segments.data(), segments.size());
    for (const auto & segment : segments) {
      auto fixture = std::make_unique<b2Fixture>(
        this, segment, std::make_unique<b2ChainShape>(chain));
      fixture->next_ = fixtures_;
      fixtures_ = fixture.get();
      owned_fixtures_.push_back(std::move(fixture));
    }
    return fixtures_;
  }

  ::b2ShapeId id = ::b2_nullShapeId;
  std::unique_ptr<b2Shape> geometry;
  switch (definition->shape->GetType()) {
    case b2Shape::e_circle: {
      auto circle = *static_cast<const b2CircleShape *>(definition->shape);
      ::b2Circle primitive = {static_cast<::b2Vec2>(circle.m_p), circle.m_radius};
      id = ::b2CreateCircleShape(id_, &shape_def, &primitive);
      geometry = std::make_unique<b2CircleShape>(circle);
      break;
    }
    case b2Shape::e_edge: {
      auto edge = *static_cast<const b2EdgeShape *>(definition->shape);
      ::b2Segment segment = {static_cast<::b2Vec2>(edge.m_vertex1),
                               static_cast<::b2Vec2>(edge.m_vertex2)};
      id = ::b2CreateSegmentShape(id_, &shape_def, &segment);
      geometry = std::make_unique<b2EdgeShape>(edge);
      break;
    }
    case b2Shape::e_polygon: {
      auto polygon = *static_cast<const b2PolygonShape *>(definition->shape);
      ::b2Vec2 vertices[b2_maxPolygonVertices];
      for (int index = 0; index < polygon.m_count; ++index) {
        vertices[index] = polygon.m_vertices[index];
      }
      auto hull = ::b2ComputeHull(vertices, polygon.m_count);
      if (hull.count < 3) {
        throw std::invalid_argument("Box2D polygon is degenerate");
      }
      auto primitive = ::b2MakePolygon(&hull, 0.0f);
      id = ::b2CreatePolygonShape(id_, &shape_def, &primitive);
      geometry = std::make_unique<b2PolygonShape>(polygon);
      break;
    }
    case b2Shape::e_chain:
      throw std::logic_error("Unreachable chain fixture");
  }
  if (!::b2Shape_IsValid(id)) {
    throw std::runtime_error("Box2D failed to create shape");
  }
  auto fixture = std::make_unique<b2Fixture>(this, id, std::move(geometry));
  ::b2Shape_SetUserData(id, fixture.get());
  fixture->next_ = fixtures_;
  fixtures_ = fixture.get();
  owned_fixtures_.push_back(std::move(fixture));
  return fixtures_;
}

b2Vec2 b2Body::GetPosition() const { return ::b2Body_GetPosition(id_); }
float b2Body::GetAngle() const
{
  auto rotation = ::b2Body_GetRotation(id_);
  return std::atan2(rotation.s, rotation.c);
}
b2Transform b2Body::GetTransform() const
{
  auto transform = ::b2Body_GetTransform(id_);
  b2Transform result;
  result.p = transform.p;
  result.q.c = transform.q.c;
  result.q.s = transform.q.s;
  return result;
}
void b2Body::SetTransform(b2Vec2 position, float angle)
{
  ::b2Body_SetTransform(id_, position, {std::cos(angle), std::sin(angle)});
  if (GetType() == b2_dynamicBody) ::b2Body_SetAwake(id_, true);
}
b2Vec2 b2Body::GetLocalPoint(b2Vec2 point) const { return ::b2Body_GetLocalPoint(id_, point); }
b2Vec2 b2Body::GetWorldPoint(b2Vec2 point) const { return ::b2Body_GetWorldPoint(id_, point); }
b2Vec2 b2Body::GetWorldVector(b2Vec2 vector) const { return ::b2Body_GetWorldVector(id_, vector); }
b2Vec2 b2Body::GetWorldCenter() const { return ::b2Body_GetWorldCenterOfMass(id_); }
b2Vec2 b2Body::GetLinearVelocity() const { return ::b2Body_GetLinearVelocity(id_); }
float b2Body::GetAngularVelocity() const { return ::b2Body_GetAngularVelocity(id_); }
b2Vec2 b2Body::GetLinearVelocityFromLocalPoint(b2Vec2 point) const
{
  return ::b2Body_GetLocalPointVelocity(id_, point);
}
float b2Body::GetLinearDamping() const { return ::b2Body_GetLinearDamping(id_); }
float b2Body::GetAngularDamping() const { return ::b2Body_GetAngularDamping(id_); }
void b2Body::SetLinearVelocity(b2Vec2 value) { ::b2Body_SetLinearVelocity(id_, value); }
void b2Body::SetAngularVelocity(float value) { ::b2Body_SetAngularVelocity(id_, value); }
void b2Body::SetAwake(bool awake) { ::b2Body_SetAwake(id_, awake); }
void b2Body::Dump() const
{
  auto position = GetPosition();
  b2Log("body type=%d position=(%g, %g) angle=%g shapes=%zu\n", GetType(),
        position.x, position.y, GetAngle(), owned_fixtures_.size());
}

b2Joint::b2Joint(b2Body * body_a, b2Body * body_b, ::b2JointId id)
: body_a_(body_a), body_b_(body_b), id_(id) {}
b2Vec2 b2Joint::GetAnchorA() const
{
  return body_a_->GetWorldPoint(::b2Joint_GetLocalAnchorA(id_));
}
b2Vec2 b2Joint::GetAnchorB() const
{
  return body_b_->GetWorldPoint(::b2Joint_GetLocalAnchorB(id_));
}
bool b2Joint::GetCollideConnected() const { return ::b2Joint_GetCollideConnected(id_); }
void b2Joint::SetUserData(void * data) { ::b2Joint_SetUserData(id_, data); }
void * b2Joint::GetUserData() const { return ::b2Joint_GetUserData(id_); }
void b2Joint::Dump() const
{
  auto anchor = GetAnchorA();
  b2Log("joint type=%d anchor=(%g, %g) connected=%d\n", GetType(), anchor.x,
        anchor.y, GetCollideConnected());
}
bool b2RevoluteJoint::IsLimitEnabled() const { return ::b2RevoluteJoint_IsLimitEnabled(id_); }
float b2RevoluteJoint::GetLowerLimit() const { return ::b2RevoluteJoint_GetLowerLimit(id_); }
float b2RevoluteJoint::GetUpperLimit() const { return ::b2RevoluteJoint_GetUpperLimit(id_); }
void b2RevoluteJoint::EnableLimit(bool enabled) { ::b2RevoluteJoint_EnableLimit(id_, enabled); }
void b2RevoluteJoint::SetLimits(float lower, float upper)
{
  constexpr float max_angle = 0.99f * std::numbers::pi_v<float>;
  ::b2RevoluteJoint_SetLimits(
    id_, std::clamp(lower, -max_angle, max_angle), std::clamp(upper, -max_angle, max_angle));
}
float b2WeldJoint::GetReferenceAngle() const { return ::b2Joint_GetReferenceAngle(id_); }
float b2WeldJoint::GetFrequency() const { return ::b2WeldJoint_GetLinearHertz(id_); }
float b2WeldJoint::GetDampingRatio() const { return ::b2WeldJoint_GetLinearDampingRatio(id_); }

b2Contact::b2Contact(b2Fixture * fixture_a, b2Fixture * fixture_b)
: fixture_a_(fixture_a), fixture_b_(fixture_b) {}
void b2Contact::GetWorldManifold(b2WorldManifold * output) const
{
  output->normal = manifold_.normal;
  for (int index = 0; index < manifold_.pointCount; ++index) {
    output->points[index] = manifold_.points[index].point;
  }
}

b2World::b2World(b2Vec2 gravity)
{
  auto definition = ::b2DefaultWorldDef();
  definition.gravity = gravity;
  id_ = ::b2CreateWorld(&definition);
  ::b2World_SetUserData(id_, this);
}
b2World::~b2World()
{
  contacts_.clear();
  joints_.clear();
  bodies_.clear();
  ::b2DestroyWorld(id_);
}
b2Body * b2World::CreateBody(const b2BodyDef * definition)
{
  auto body_def = ::b2DefaultBodyDef();
  body_def.type = definition->type;
  body_def.position = definition->position;
  body_def.rotation = {std::cos(definition->angle), std::sin(definition->angle)};
  body_def.linearDamping = definition->linearDamping;
  body_def.angularDamping = definition->angularDamping;
  auto body = std::make_unique<b2Body>(this, ::b2CreateBody(id_, &body_def));
  b2Body * result = body.get();
  bodies_.push_back(std::move(body));
  return result;
}
void b2World::DestroyBody(b2Body * body)
{
  EndContactsFor(body);
  for (auto joint = joints_.begin(); joint != joints_.end();) {
    if ((*joint)->body_a_ == body || (*joint)->body_b_ == body) {
      joint = joints_.erase(joint);
    } else {
      ++joint;
    }
  }
  ::b2DestroyBody(body->id_);
  std::erase_if(bodies_, [body](const auto & candidate) { return candidate.get() == body; });
}
b2Joint * b2World::CreateJoint(const b2JointDef * definition)
{
  ::b2JointId id = ::b2_nullJointId;
  std::unique_ptr<b2Joint> joint;
  if (auto revolute = dynamic_cast<const b2RevoluteJointDef *>(definition)) {
    auto joint_def = ::b2DefaultRevoluteJointDef();
    joint_def.bodyIdA = revolute->bodyA->id_;
    joint_def.bodyIdB = revolute->bodyB->id_;
    joint_def.localAnchorA = revolute->localAnchorA;
    joint_def.localAnchorB = revolute->localAnchorB;
    joint_def.enableLimit = revolute->enableLimit;
    constexpr float max_angle = 0.99f * std::numbers::pi_v<float>;
    joint_def.lowerAngle = std::clamp(revolute->lowerAngle, -max_angle, max_angle);
    joint_def.upperAngle = std::clamp(revolute->upperAngle, -max_angle, max_angle);
    if (joint_def.lowerAngle != revolute->lowerAngle ||
        joint_def.upperAngle != revolute->upperAngle) {
      b2Log("Box2D 3.1 clamps revolute limits [%g, %g] to [%g, %g] radians\n",
            revolute->lowerAngle, revolute->upperAngle,
            joint_def.lowerAngle, joint_def.upperAngle);
    }
    joint_def.collideConnected = revolute->collideConnected;
    id = ::b2CreateRevoluteJoint(id_, &joint_def);
    joint = std::make_unique<b2RevoluteJoint>(definition->bodyA, definition->bodyB, id);
  } else if (auto weld = dynamic_cast<const b2WeldJointDef *>(definition)) {
    auto joint_def = ::b2DefaultWeldJointDef();
    joint_def.bodyIdA = weld->bodyA->id_;
    joint_def.bodyIdB = weld->bodyB->id_;
    joint_def.localAnchorA = weld->localAnchorA;
    joint_def.localAnchorB = weld->localAnchorB;
    joint_def.referenceAngle = weld->referenceAngle;
    joint_def.linearHertz = weld->frequencyHz;
    joint_def.linearDampingRatio = weld->dampingRatio;
    joint_def.angularHertz = weld->frequencyHz;
    joint_def.angularDampingRatio = weld->dampingRatio;
    joint_def.collideConnected = weld->collideConnected;
    id = ::b2CreateWeldJoint(id_, &joint_def);
    joint = std::make_unique<b2WeldJoint>(definition->bodyA, definition->bodyB, id);
  } else {
    throw std::invalid_argument("Unsupported joint type");
  }
  b2Joint * result = joint.get();
  joints_.push_back(std::move(joint));
  return result;
}
void b2World::DestroyJoint(b2Joint * joint)
{
  ::b2DestroyJoint(joint->id_);
  std::erase_if(joints_, [joint](const auto & candidate) { return candidate.get() == joint; });
}

b2World::ContactKey b2World::Key(b2Fixture * first, b2Fixture * second)
{
  return std::less<b2Fixture *>{}(first, second) ? ContactKey{first, second}
                                                : ContactKey{second, first};
}
b2Fixture * b2World::FindFixture(::b2ShapeId id) const
{
  if (::b2Shape_IsValid(id)) {
    if (auto * fixture = static_cast<b2Fixture *>(::b2Shape_GetUserData(id))) return fixture;
  }
  for (const auto & body : bodies_) {
    for (const auto & fixture : body->owned_fixtures_) {
      if (fixture->id_.index1 == id.index1 && fixture->id_.generation == id.generation &&
          fixture->id_.world0 == id.world0) return fixture.get();
    }
  }
  return nullptr;
}
void b2World::EndContactsFor(b2Body * body, b2Fixture * fixture)
{
  for (auto contact = contacts_.begin(); contact != contacts_.end();) {
    bool touches_body = contact->second->fixture_a_->GetBody() == body ||
                        contact->second->fixture_b_->GetBody() == body;
    bool touches_fixture = fixture == nullptr || contact->second->fixture_a_ == fixture ||
                           contact->second->fixture_b_ == fixture;
    if (touches_body && touches_fixture) {
      if (listener_) listener_->EndContact(contact->second.get());
      contact = contacts_.erase(contact);
    } else {
      ++contact;
    }
  }
}
void b2World::Step(float time_step, int substeps)
{
  ::b2World_Step(id_, time_step, substeps);
  if (!listener_) return;

  auto begin = [this](b2Fixture * first, b2Fixture * second, const b2Manifold * manifold) {
    if (!first || !second) return;
    if (first->GetBody() == second->GetBody()) return;
    for (const auto & joint : joints_) {
      bool connected =
        (joint->body_a_ == first->GetBody() && joint->body_b_ == second->GetBody()) ||
        (joint->body_b_ == first->GetBody() && joint->body_a_ == second->GetBody());
      if (connected && !joint->GetCollideConnected()) return;
    }
    auto key = Key(first, second);
    if (contacts_.contains(key)) return;
    auto contact = std::make_unique<b2Contact>(first, second);
    if (manifold) contact->manifold_ = *manifold;
    auto * value = contact.get();
    contacts_.emplace(key, std::move(contact));
    listener_->BeginContact(value);
  };
  auto end = [this](b2Fixture * first, b2Fixture * second) {
    if (!first || !second) return;
    auto match = contacts_.find(Key(first, second));
    if (match == contacts_.end()) return;
    listener_->EndContact(match->second.get());
    contacts_.erase(match);
  };

  auto contact_events = ::b2World_GetContactEvents(id_);
  for (int index = 0; index < contact_events.beginCount; ++index) {
    const auto & event = contact_events.beginEvents[index];
    bool touching = false;
    for (int point = 0; point < event.manifold.pointCount; ++point) {
      touching |= event.manifold.points[point].separation <= 0.0f;
    }
    if (touching) begin(FindFixture(event.shapeIdA), FindFixture(event.shapeIdB), &event.manifold);
  }
  auto sensor_events = ::b2World_GetSensorEvents(id_);
  for (int index = 0; index < sensor_events.beginCount; ++index) {
    const auto & event = sensor_events.beginEvents[index];
    begin(FindFixture(event.sensorShapeId), FindFixture(event.visitorShapeId), nullptr);
  }
  for (int index = 0; index < contact_events.endCount; ++index) {
    const auto & event = contact_events.endEvents[index];
    end(FindFixture(event.shapeIdA), FindFixture(event.shapeIdB));
  }
  for (int index = 0; index < sensor_events.endCount; ++index) {
    const auto & event = sensor_events.endEvents[index];
    end(FindFixture(event.sensorShapeId), FindFixture(event.visitorShapeId));
  }

  auto & data = contact_data_;
  for (const auto & body : bodies_) {
    if (body->GetType() != b2_dynamicBody) continue;
    int capacity = ::b2Body_GetContactCapacity(body->id_);
    if (capacity == 0) continue;
    data.resize(capacity);
    int count = ::b2Body_GetContactData(body->id_, data.data(), capacity);
    for (int index = 0; index < count; ++index) {
      bool touching = false;
      for (int point = 0; point < data[index].manifold.pointCount; ++point) {
        touching |= data[index].manifold.points[point].separation <= 0.0f;
      }
      if (!touching) continue;
      auto * first = FindFixture(data[index].shapeIdA);
      auto * second = FindFixture(data[index].shapeIdB);
      if (!first || !second) continue;
      if (first->GetBody()->GetType() == b2_dynamicBody && first->GetBody() != body.get()) {
        continue;
      }
      auto match = contacts_.find(Key(first, second));
      if (match == contacts_.end()) {
        begin(first, second, &data[index].manifold);
        match = contacts_.find(Key(first, second));
      }
      if (match == contacts_.end()) continue;
      b2Contact * contact = match->second.get();
      contact->manifold_ = data[index].manifold;
      if (contact->fixture_a_ != first) {
        contact->manifold_.normal.x = -contact->manifold_.normal.x;
        contact->manifold_.normal.y = -contact->manifold_.normal.y;
      }
      b2ContactImpulse impulse;
      impulse.count = contact->manifold_.pointCount;
      for (int point = 0; point < impulse.count; ++point) {
        impulse.normalImpulses[point] = contact->manifold_.points[point].normalImpulse;
        impulse.tangentImpulses[point] = contact->manifold_.points[point].tangentImpulse;
      }
      listener_->PostSolve(contact, &impulse);
    }
  }
}
void b2World::RayCast(b2RayCastCallback * callback, b2Vec2 start, b2Vec2 end) const
{
  struct CastContext { const b2World * world; b2RayCastCallback * callback; };
  CastContext context{this, callback};
  auto filter = ::b2DefaultQueryFilter();
  filter.categoryBits = UINT64_MAX;
  ::b2World_CastRay(
    id_, start, end - start, filter,
    [](::b2ShapeId shape, ::b2Vec2 point, ::b2Vec2 normal, float fraction, void * data) {
      if (fraction == 0.0f) return -1.0f;
      auto * cast = static_cast<CastContext *>(data);
      auto * fixture = cast->world->FindFixture(shape);
      return fixture ? cast->callback->ReportFixture(fixture, point, normal, fraction) : -1.0f;
    }, &context);
}
void b2Log(const char * format, ...)
{
  va_list args;
  va_start(args, format);
  std::vfprintf(stderr, format, args);
  va_end(args);
}

}  // namespace flatland