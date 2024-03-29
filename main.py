#!python
#!/usr/bin/env python
from kivy.app import App
from kivy.uix.bubble import Bubble
from kivy.animation import Animation
from kivy.uix.floatlayout import FloatLayout
from kivy.lang import Builder
from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.graphics.texture import Texture
import cv2
import numpy as np
import serial
import time
Builder.load_string('''
#template for menu items
[ListButton@ToggleButton]
    background_down: 'atlas://data/images/defaulttheme/bubble_btn'
    background_normal: 'atlas://data/images/defaulttheme/bubble_btn_pressed'
    group: 'context_menue_root'
    on_release: ctx.on_release(self) if hasattr(ctx, 'on_release') else None
    size_hint: ctx.size_hint if hasattr(ctx, 'size_hint') else (1, 1)
    width: ctx.width if hasattr(ctx, 'width') else 1
    text: ctx.text if hasattr(ctx, 'text') else ''
    Image:
        source: ctx.btn_img if ctx.text == 'hows' \
            else 'atlas://data/images/defaulttheme/bubble_btn'
        size: (20, 20)
        y: self.parent.y + (self.parent.height/2) - (self.height/2)
        x: self.parent.x + (self.parent.width - self.width)

<Test>
    Button:
        text: 'press to launch menu'
        size_hint: .2, .2
        on_release:  root.add_menu(args[0])

<Cmenu>
    size_hint: None, None
    size: 120, 250
    pos: (5, 50)
    padding: 5
    background_color: .2, .9, 1, .7
    #wanna have some fun? set this to 'data/images/image-loading.gif'
    background_image: 'atlas://data/images/defaulttheme/button_pressed'
    orientation: 'vertical'
    BoxLayout:
        padding: 5
        ScrollView:
            BoxLayout:
                size_hint: None, 1
                width: root.width * 2 - 40
                #root menu add/edit items here to show them in root menu
                BoxLayout:
                    orientation: 'vertical'
                    ListButton:
                        text: 'Hello'
                        on_release: root.menu_selected
                    ListButton:
                        text: 'World'
                        on_release: root.menu_selected
                    ListButton:
                        text: 'hows'
                        #'>'image
                        btn_img: 'atlas://data/images/defaulttheme/tree_closed'
                        on_release: root.menu_selected
                    ListButton:
                        text: 'it'
                        on_release: root.menu_selected
                    ListButton:
                        text: 'going'
                        on_release: root.menu_selected
                    ListButton:
                        text: 'peace'
                        on_release: root.menu_selected
                # end root menu
                #sub-menu
                BoxLayout:
                    ListButton:
                        # go back(root menu) button
                        text: '<'
                        size_hint: (.15, 1)
                        on_release: root.menu_selected
                    BoxLayout:
                        orientation: 'vertical'
                        ListButton:
                            text: 'The'
                            on_release: root.menu_selected
                        ListButton:
                            text: 'revolving'
                            on_release: root.menu_selected
                        ListButton:
                            text: 'door'
                            on_release: root.menu_selected
                        ListButton:
                            text: 'hits'
                            on_release: root.menu_selected
                        ListButton:
                            text: 'every'
                            on_release: root.menu_selected
                        ListButton:
                            text: 'one'
                            on_release: root.menu_selected
                #end sub-menu
''')


class Cmenu(Bubble):

    def menu_selected(self, *l):
        if l[0].text == 'hows':
            # move to sub menu
            Animation(scroll_x = 1, d=.5 ).start(l[0].parent.parent.parent)
            #l[0].parent.parent.parent change this and everything relative to something non-relative if you want-to make the menu more extensible
        elif l[0].text == '<':
            # move back to root menu
            Animation(scroll_x = 0, d=.5 ).start(l[0].parent.parent.parent)
        else:
            #fade out animation
            (r, g, b, a) = self.parent.context_menu.background_color

            def on_anim_complete(*l):
                self.parent.context_menu.background_color = (r, g, b, a)
                self.parent.remove_widget(self.parent.context_menu)

            anim = Animation(background_color = (0, 0, 0, 0), d=.1 )
            anim.start(self.parent.context_menu)
            anim.bind(on_complete = on_anim_complete)
            print (l[0].text + ' selected')


class Test(FloatLayout):

    def __init__(self, **kwargs):
        super(Test, self).__init__(**kwargs)

    def on_touch_down(self, *l):
        #allow kids to get touch
        if super(Test, self).on_touch_down(*l):
            return True
        # remove menu when touched and menu exists
        if hasattr(self, 'context_menu'):
            self.remove_widget(self.context_menu)

    def add_menu(self, obj, *l):
        if not hasattr(self, 'context_menu'):
            self.context_menu = Cmenu()
        self.remove_widget(self.context_menu)
        self.add_widget(self.context_menu)
        self.context_menu.pos = obj.pos[0]+ obj.width, obj.pos[1]

class MyApp(App):
    def build(self):
        self.img1=Image()
        layout = BoxLayout()
        layout.add_widget(self.img1)
        #cv2
        windowname = 'Result'
        cv2.namedWindow(windowname)
        self.capture = cv2.VideoCapture(0)
        Clock.schedule_interval(self.update, 1.0/33.0)
        Test()
        return layout

    def update(self, dt):
        # display image from cam in opencv window
        ret, frame = self.capture.read()
        cv2.imshow("CV2 Image", frame)
        # convert it to texture
        buf1 = cv2.flip(frame, 0)
        buf = buf1.tostring()
        texture1 = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr') 
        #if working on RASPBERRY PI, use colorfmt='rgba' here instead, but stick with "bgr" in blit_buffer. 
        texture1.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
        # display image from the texture
        self.img1.texture = texture1

if __name__ == '__main__':
    MyApp().run()
    cv2.destoryAllWindows()