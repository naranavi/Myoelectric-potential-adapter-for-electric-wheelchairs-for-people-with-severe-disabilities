
## Title
重度の四肢障害を持つ人々のための電動車いす筋電位アダプター。
Electric Wheelchair Myoelectric Potential Adapter for People with Severe Limb Disorders.

##
[簡単な説明動画 Brief explanation video]( https://youtu.be/57T5eTsrPN8)
## Inspiration
重度四肢障碍者の方の自分で動きたい欲求は私が考えるよりはるかに強いことに気づきました。
I have noticed that people with severe limb disabilities have a much stronger desire to move on their own than I think.

## What it does
手足、首が不自由でも現在ある程度自由に動かせる口の動きで電動車いすを操縦することを考えました。  
奥歯を噛みしめることにより電動車いすを自在に操縦します。
I thought about operating an electric wheelchair with the movement of the mouth, which allows me to move freely to some extent even if my limbs and neck are inconvenient.
You can freely control an electric wheelchair by biting your back teeth.

## How we built it
奥歯を噛みしめたときに発生する側頭筋の筋電位をコメカミ付近に電極を置いて検出し、その信号を加工して電動車いすを動作させる信号に変換しました。  
ジョイスティックを改造して接続していたのを車いすメーカのインターフェースに合わせました。   
・無線式筋電位アンプの開発   
・筋電位信号受信、操縦アルゴリズム実行及び電動車いすインターフェース接続のためのアダプタ開発   
The myoelectric potential of the temporalis muscle generated when the back teeth are bitten is detected by placing an electrode near the temple, and the signal is processed and converted into a signal to operate the electric wheelchair.   
The joystick was modified and connected to match the interface of the wheelchair manufacturer.   
・ Development of wireless myoelectric potential amplifier   
・ Development of adapters for receiving myoelectric potential signals, executing maneuvering  
  algorithms, and connecting electric wheelchair interfaces

## Challenges we ran into
左右のコメカミから検出した2つの信号で電動車いすを自在に操縦するアルゴリズムをどのようにするか悩みました。   
コマンド方式すると多くの動作を選択することができますがリアルタイム性が犠牲になります。   
右奥歯を噛みしめれば右に旋回、左奥歯を噛みしめれば左旋回、両方で前進、歯を噛みしめている間動いて力を緩めると停止する、直観的な操縦方法になりました 。  
I was worried about how to use the algorithm to freely control the electric wheelchair with the two signals detected from the left and right temples.   
The command method allows you to select many actions, but at the expense of real-time performance.
If you bite the right back tooth, it turns to the right, if you bite the left back tooth, it turns left, both move forward, and if you run while biting the tooth and loosen the force, it stops.   
It became an intuitive maneuvering method.   

## Accomplishments that we're proud of
使用者が今まで見学だった体育の授業に参加できるようになりました。   
使用者と一緒に開発することにより想定以上の車いすの操作性能を得ることができました。   
使用者の努力で前後とその場旋回の動きの予定が斜め方向が入ってS字カーブを走行できるようになりました。   
学校内で電動車いすを運転する免許を取得することができました。   
・通常の電動車いすの速度で操縦可能   
・S字コースも通常速度でクリア   
・体育の授業に参加   
Users can now participate in physical education classes.   
By developing it together with the user, we were able to obtain more wheelchair operation performance than expected.   
Thanks to the efforts of the user, it has become possible to drive on an S-shaped curve with diagonal directions in the front-back and in-situ turning movement schedules.   
I was able to get a license to drive an electric wheelchair in the school.   
・ Can be operated at normal electric wheelchair speed   
・ Clear S-shaped course at normal speed   
・ Participate in physical education classes    

## What we learned
開発者がここまでだろうと勝手にスペックの範囲を決めてしまわずに人間の可能性を信じること   
（ゆっくりでも前進、後進、旋回ができればOKだろうなど）   
The developer should not decide the range of specifications.   
Believe in the potential of the parties and carry out development   

## What's next for Team 5-ADL-Track4
現在、長時間の運転を行うと顎が疲れるので安全で疲れない操作アルゴリズムの開発。   
あと重度四肢障碍者がアクションゲームをこなせるインターフェースの開発をしたいです。   
Currently, we are developing a safe and comfortable operation algorithm because the jaw gets tired when driving for a long time.   
I also want to develop an interface that allows people with severe limb disabilities to play action games.   
