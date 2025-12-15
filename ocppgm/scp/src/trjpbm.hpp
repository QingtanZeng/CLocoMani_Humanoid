
/* @brief 顶层轨迹优化问题 */
//Constructor from specific DynMdl SubClass
template<typename MdlType>
class TrjPbm{
public:
    MdlType dynmdl;
    DynCstr<MdlType> dyncstr;



};