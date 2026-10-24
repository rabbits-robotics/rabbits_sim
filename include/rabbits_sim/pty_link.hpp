// 疑似端末（pty）の組を作り、こちら側（master）を読み書きする。
// もう片側（slave、/dev/pts/N）へのシンボリックリンクを link_path に作るので、robot_bridge は
// そのパスを device に指定すれば実機のシリアルと同じように開ける。
// slave の fd も自分で開いたまま持つ: bridge が slave を閉じても pty が消えず、master の read が EIO にならない
#ifndef RABBITS_SIM__PTY_LINK_HPP_
#define RABBITS_SIM__PTY_LINK_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace rabbits_sim
{

class PtyLink
{
public:
  PtyLink() = default;
  ~PtyLink();
  PtyLink(const PtyLink &) = delete;
  PtyLink & operator=(const PtyLink &) = delete;

  // openpty → slave を raw、master をノンブロッキング、link_path → slave_path のリンク（既存は置き換え）
  bool Open(const std::string & link_path);
  void Close();  // リンクを消し、fd を閉じる
  bool is_open() const {return master_fd_ >= 0;}
  const std::string & slave_path() const {return slave_path_;}
  const std::string & link_path() const {return link_path_;}

  // 読めたバイト数。無ければ 0。EAGAIN / EIO は 0。致命的なら -1
  int Read(uint8_t * buf, size_t len);
  // 書けたバイト数。相手が読んでいなくて書けなければ 0（ブロックしない）
  int Write(const uint8_t * buf, size_t len);

private:
  int master_fd_ = -1;
  int slave_fd_ = -1;
  std::string slave_path_;
  std::string link_path_;
};

}  // namespace rabbits_sim

#endif  // RABBITS_SIM__PTY_LINK_HPP_
