#include <physx_pack/visibility_query.h>
int main()
{
  static_assert(physx_pack::ApiVersion == 2);
  physx_pack::QueryContext Context;
  return physx_pack::ToString(physx_pack::Error::None) ? 0 : 1;
}
