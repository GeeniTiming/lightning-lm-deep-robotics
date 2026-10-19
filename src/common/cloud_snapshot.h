#ifndef LIGHTNING_CLOUD_SNAPSHOT_H
#define LIGHTNING_CLOUD_SNAPSHOT_H
#include <mutex>
#include "common/point_def.h"
namespace lightning {
// Publish only from the cloud's owning writer after a frame is complete.
// Readers never receive the writer's mutable buffer or the cached snapshot.
class CloudSnapshot {
 public:
  void Publish(const PointCloudType& completed) {
    CloudPtr next(new PointCloudType(completed));
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_ = next;
  }
  CloudPtr Copy() const {
    CloudPtr snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot = cloud_;
    }
    return snapshot ? CloudPtr(new PointCloudType(*snapshot)) : CloudPtr();
  }
 private:
  mutable std::mutex mutex_;
  CloudPtr cloud_;
};
}
#endif
