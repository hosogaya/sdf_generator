# Sdf generator 
[voxfield](https://github.dev/VIS4ROB-lab/voxfield)を参考に作成している．

# Common Type 
* `AnyIndex`: 3次元のインデックス
* `AnyIndexHash`: AnyIndexに対するハッシュ値を返す構造体
* `AnyIndexHashMap<ValueType>`: AnyIndexに対するValueTypeの `std::unordered_map`を定義するもの．std::vectorで定義するよりも`AnyIndexHash`によるハッシュ値でのアクセスの方が早いためこれが実装されていると考えれれる．


# Structure
* `Voxel`: ３次元の立方体．これに距離だとか勾配が保存されている．
* `Block`: 大きめの立方体にVoxelを敷き詰めたもの．Voxelは一次元配列で定義されており，`z, y, x`の順番でループを回してアクセするような形となっている．
* `Layer`: Blockに対する`std::unordered_map`を持っている．


# Point Cloud Process
* 観測したポイントクラウドから周囲環境（サーフェース）の法線ベクトルを計算するために一度２次元画像に落とし込んでいる．
