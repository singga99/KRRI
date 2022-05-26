## Image Transport

##### repub_img

* 터미널창에서 실행

```
$ ros2 run image_transport republish compressed --ros-args --remap in/compressed:=compressed_image_frontleft_output --ros-args --remap out:=image_out
```

*  compressed_image_frontleft_output : 압축형태로 발행되는 토픽

* mage_out : 새롭게 발행할 토픽이름