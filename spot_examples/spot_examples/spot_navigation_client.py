import argparse
import logging
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.utilities import fqn, namespace_with
from rclpy.node import Node
import time

from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.action import NavigateTo  # type: ignore
from spot_msgs.srv import GraphNavInitialize
from spot_msgs.srv import ListGraph

from spot_examples.simple_spot_commander import SimpleSpotCommander


class NavigateToWaypoint:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._robot_name = robot_name
        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )
        self._robot_navigation_client = ActionClientWrapper(
           NavigateTo, namespace_with(self._robot_name, "navigate_to"), node
        )
        self._grap_nav_client = node.create_client(GraphNavInitialize, "/" + self._robot_name + "/graph_nav_initialize")
        self._waypoint_client = node.create_client(ListGraph, "/" + self._robot_name + "/list_graph")

        while not self._grap_nav_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Service not available, waiting...')

        while not self._waypoint_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Service not available, waiting...')

        self._logger.info('Service Found')

    def initialize_robot(self, args) -> bool:
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot message was " + result.message)
            return False
        self._logger.info("Claimed robot")
        print("here")
        # Stand the robot up.
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot message was " + result.message)
            return False
        self._logger.info("Standing robot up")
        result = self._robot.command("stand")
        if not result.success:
            self._logger.error("Robot did not stand message was " + result.message)
            return False
        self._logger.info("Successfully stood up.")

        self.waypoint_index = ['blocky-racoon-3zC9eC.f+shO88yV+YYCAQ==', 'west-manx-pgX8WRuKBAfyyFzH2ISZTg==', 'wily-puppy-R2RJb2qdeyHsFv4FshVWcA==', 'sparse-grub-FcJ7icFJF5x0+QVuZFcSBw==', 'inured-ocelot-eZmWLuZbhcHXaSR8IIUDCw==', 'healed-mare-E83fLVbpkk+V5iW4FkpXMg==', 'four-mite-qhYO5uIiV.9vZ1qbieW9LA==', 'erring-rhino-7zd+U4TOgjM6cFpMm0b9pA==', 'rifled-drill-CiZ8r5W4dQjMiRpeTa1sfQ==', 'lento-wasp-ikmYGMw8pINrfEWoEYrfGg==', 'sallow-foal-MUrj+XWA0+39DeAhlEiXTg==', 'odious-beaver-w1PYzh6PWIpEUykDkxwT.w==', 'reflex-mouse-2COTQnPOpRmQCxXb1xiPTA==', 'salmon-drum-euMGMeIl3EHX7lfNWuzQRA==', 'marred-hornet-QBYDJrQOjDX8XpqOAPVyCQ==', 'unsaid-hawk-6paziHHE8jSOzyzYYnGHHQ==', 'piano-syphon-eSmyomnND.RfaI3yUMkusg==', 'piggy-conger-U9mH4oONxZUogcbIrJaTnA==', 'biform-midge-4VjhSq2ewC0i8v9fF14zNA==', 'bats-mite-bOw.yJdV6SMgPGSnjAxyMQ==', 'gemmed-lizard-AJBMY.eEnOOqhaIN5+CBRQ==', 'axial-mouse-rKVAVGiBMLLq89RfAC5GXQ==', 'formed-deer-fdNI1mqvNRuQVvEfKLvF7A==', 'barky-civet-Zu.d5xcDJ3ekOlgzdCwheA==', 'goofy-locust-IAR0wtNA5bh98E5EKFkCaA==', 'manic-prawn-dgyp1Q37ZA4ZU5a3gJcw7g==', 'fifty-oxtail-BBBk2UoUzPhwdDgMNOEsdA==', 'sleepy-agouti-E.fq9nZ9qLuTo7cbq6fZJg==', 'onside-macaw-U8W3cAK4vEQ8jYzDb67Cvg==', 'tardy-lizard-sjhx8dg.eKcp7SEwRh5cxQ==', 'nubby-lapdog-SPD7zSOOQp5KKDeX8Iy+ww==', 'wound-heron-HE66UidU++urVSo2P7USow==', 'affine-bedbug-Y81pDzLIsxY5aGDDn4onpg==', 'stocky-amoeba-GhJXAQeEkEq1qfrw7ln8xw==', 'braggy-skunk-+LygQf+SGerWkxG462c6eg==', 'upwind-drum-nbpk3GfxAqmEXuriBvCb0Q==', 'spiked-goat-lTiUUjMkLV4Qr0JiNyL.IQ==', 'flint-snake-Ar0jk9LoBjS+eb3lAczJ5A==', 'lobed-merino-8ZhWkb4qpLCLg7peExNFBQ==', 'dummy-clam-PDT3BBDUkmzqXtYUbJeLEw==', 'bicorn-moray-fWBxfZ4doDBJyesX.KJibQ==', 'loco-kudu-uI+7goJWFnuK.dZPwhMlrg==', 'curled-bobcat-6eOWDDxn6k.ul4N+BKbDRA==', 'crispy-taipan-E8x7jkQ+Ia0ylHyWk1cotw==', 'arced-afghan-igi.XMybz4o.kThRbrg9MQ==', 'famed-equid-jIuAdGzVjhLClWT2t+ZocQ==', 'roily-antler-+z1cIf9f0DqpNFAmvePtlw==', 'shaped-borzoi-ka+ALkYE21ZDb4agFL37Bw==', 'midi-hippo-0D72B8UDWsM5U081icPa2g==', 'rummy-otter-zkriFE42B534v.A.mODgfg==', 'padded-agouti-juZJznMOsBr+QMYMUl1kgg==', 'wonted-ant-9N6KDiVte801K5gjZS5Bvw==', 'snarly-eagle-YVhLpcwMWTkKFp3D+VatAw==', 'fly-ape-vw7zbh8h8o3WkyGRV+PH+w==', 'aboral-vole-dgqhUmy800Z2H7xS6RA7dA==', 'naming-seal-wAmlHLb+WgVvSZiu.rkSQg==', 'astute-wolf-tBTscM2wRvhmdULNye8ntQ==', 'hazel-tiger-69KgzibRll9tEiGRTx.ohw==', 'grazed-vermin-YpDTxi6smnKFz+M0.yzIDg==', 'dapper-ocelot-GKwecaXKuF6DekrSvBs7rQ==', 'boiled-tsetse-gHEHTcZlddxn3zneYOn0eg==', 'ashy-koala-dbTe76vLpL6vtEuB+RFo.Q==', 'aboral-beef-G0enT3id3cTpZLovvi.Gjg==', 'aguish-sow-5YzlnpuE3VqZvPvVyhh6LA==', 'fain-eel-Zkj3tw.ih3T58W4b17BZ9g==', 'honey-raven-i.8qPQPLYe+wkq1tiClDpg==', 'going-crest-.xqdkMHmzcxuGtKyQsw7tA==', 'gonzo-donkey-NR2cBjfluzFClmaCKzbjJg==', 'cooked-lion-YfTFoePAz+85zM0BDAF6GA==', 'cloven-snake-k.EiG6j2PWJXojUbwJZd9g==', 'color-earwig-z4BD+xeUW5qUBLMgvbNtww==', 'enate-lapdog-p+FNeO8NBArpK3nVNHruNA==', 'scant-brahma-Du1Z9weFUB60tEBkB96dKw==', 'bowery-flea-z4qgLWbyiAXe8RMm8n1mLw==', 'affine-mayfly-N6K8VlQBOwJJuAW5+dr.rQ==', 'gilt-python-LJRYZIXnCYi9j9deZZw9Jw==', 'tasty-crake-0RyqdWu+AIKsqO07cRXrHg==', 'rueful-doe-XEb4dEMRSeY5EmMHXDkfmg==', 'timed-hybrid-BU01mNZdldPdlHdstSx0fA==', 'pent-gull-NO2KKpwZpC1i5QNWhcMGgw==', 'six-merlin-bBz+SnxWENGIZVGMUhaDNQ==', 'naiant-hogg-KNUjhzUG8iR16Yr08Ot0kw==', 'tinkly-sawfly-EDFXknVckYdLWG.skFVOYA==', 'warped-cobra-PPrm+6e5eBVlGv6zqHC.Rw==', 'sunken-diatom-ZEaIkXjKy.tPF8efekx7Cg==', 'edgy-vixen-Mg4bPioKChzrr7QlRUZEvQ==', 'leal-snipe-WNpWj2gDr1v++joY28UNNA==', 'pawky-crow-URaNNKAuEOQvHCeIX6KwRQ==', 'mantic-tomcat-gf6hUpGo3r0UDBLH4UtXbQ==', 'salted-toucan-2w+Ca8Nu0gDPuPfpv74qEw==', 'naming-oryx-COzlJ.H0+vKLsExajaUd1w==', 'spiral-condor-0aqMGoPQNlMIcOGv16GDyw==', 'afire-giant-6L5s4Rnxuq2SoQI.HAsu.Q==', 'scotch-tick-3Mq9uQI3R1Ll6sNgZnkTyg==', 'dummy-rook-z8ED3oaxn4xgBuxTWLeKOA==', 'eyed-duck-ZHN0DMh9xTrw27KQax8CeQ==', 'modish-llama-EaSQRkKxF5AXeyKVt9Ua9Q==', 'stubby-buck-emFDDCf4U2YQ+J3iQIp.OQ==', 'caring-doe-duk1Mlu4F.T7S7AsNWF1+Q==', 'mum-maggot-bfj8vCO7rWqyAQoJ+4vG.w==', 'bowing-marmot-I2gcFabVoS82CZVaewgZWg==', 'mouldy-maggot-tQ6.jMX3ccnO4YWvcCMPvg==', 'velar-hare-UlGBlz92ViB5ZDKBnlJI0Q==', 'rueful-drum-NtAW4+k8T+B9nqjpdYhxuA==', 'hoarse-grub-aptOYCXq7HDmYZJvuhYRHg==', 'dashed-seal-f1pxa6BiKaVgzfa1l62+.g==', 'tasty-spider-Sns33XQctsx+JScpknFkFw==', 'dated-weevil-oGdv2p5DM+jeNbgPlSuWjg==', 'sudden-syphon-7gs.O4bqzCpJQ0a.7xVajg==', 'nearby-hare-enlA.Paq1STxkDZ2qacZGw==', 'cyan-clam-M971xa54aGXNWrdl1hjMmQ==', 'coiled-goby-DsnmLmuomsPpcdQDgrQgcw==', 'hatted-moray-gNeDkXNNFMQ96POdCrmOfg==', 'joyous-hare-LfVeV2fAeJFR5jp6EU4Lyw==', 'dipped-pincer-pXlE4l.uLaI9lswlyJfFyA==', 'corked-hippo-oFksyYJfODCFtVL5OyDBKw==', 'pied-tapir-SaPCQxCkNrPMPS.ai4CgoQ==', 'tetchy-toad-hk13DGWEqFDhu9WJdJBgGg==', 'beetle-beetle-4E8+Bu+ILZ6z0t7f8KR6+Q==', 'known-coyote-rfs8LSgsAujHxJfyGhJbMw==', 'bedrid-howler-xigf.ZA0hHgEyN0dMkLkuw==', 'scarce-otus-nra234asbOrKK6ZXMVhRUA==', 'pied-antler-tTTJqP.Hv1ulqt0pwl+z3g==', 'alight-rodent-tEM71lI8AVJvdj9MzdlOFg==', 'funded-marlin-+G7gdRy0wH2UbyC9q3yLkg==', 'wedged-tern-t0Ew+HY3MxIF90CAr1XbWg==', 'scrub-setter-Tm4yfD3jRASbXiutq.iQ4g==', 'manky-merlin-z8eBHny4ijmQiCD4wNC71w==', 'known-manx-BBeAV.ckITxozNxO8vABSA==', 'crispy-cicala-GAdTUBO2RSgvUo9sidkmwA==', 'cowled-jaguar-ufwdci6kETfOvIkd+tbUXw==', 'extant-taipan-tEDYLWTcG8qYT75RiUtTyQ==', 'boss-tick-MNG+Ce.tf6sT6svA6gGbjQ==', 'agreed-tarpan-08HLWp9rCQdYfqnu3A2hzw==', 'bibbed-virus-cY3wE89h51GCdzGt5mYk7Q==', 'lee-mole-hRCZV7jxY++Eu1jeggvYVQ==', 'loony-sow-3Y2YlUrj2uIXsOHclCDeBQ==', 'epic-gaur-wab4Kg3F5HzcafL5bI2krw==', 'proto-seal-+zQyM8H7ldSGNzj9ldPSQw==', 'upmost-ant-LVarWLRROhjPq91CyF8Gkw==', 'leased-sponge-SoBR7u2eG3qKKoNoa79i+g==', 'balky-weaver-mOlOyzEDRNGv7VlF1.k.nw==', 'sleepy-spider-OdtojOrWODe1ONzVpMamGw==', 'kempt-spawn-sDTIRwLmyKBnUft07WeerA==', 'owned-whale-K2CN0JqT.JQCR+EYWFgopA==', 'jet-howler-gtUz+XwARW8sZeCmWAELbQ==', 'anemic-carp-itNm0eFAo5wZNUUEHeploA==', 'lobed-shrew-lXEVU0eAAXto98GALxlPWA==', 'aft-vermin-lIUBQupHI3H81Dh29badgg==', 'tenth-snail-ba4q72QLtS.XlyIqQG8Vng==', 'jailed-hornet-kHoVJ2gMLCjGe5aIiL8WAw==', 'mighty-chetah-1bvowGUPgMLEPXXQkUQBMQ==', 'famous-corgi-Fs0YfsRUtuZ7Cg8cB2c.nA==', 'drafty-egg-xnizNEzB+9xNLpc0ZguGnw==', 'proto-ape-EGnkCwmVheEeupTklGQm0g==', 'macho-poodle-YqNR+gGp6j21wJc74a6K9g==', 'haired-tern-7Ns0T5M5nnGajhj09UzjnQ==', 'beetle-goat-nze7v7nIwQNoKioBUNjsoQ==', 'deific-goby-58iObi4xte0uG+uLBKRuWw==', 'afoul-cayman-qxIsPBKj9Ke7XG58g4KTTw==', 'acting-dog-3rh4ZHDGePJYcmPO+AZIAw==', 'prize-gadfly-+jSmbw7mcQddZB6BIzJ3cA==', 'pro-tsetse-4pvcDSPU6FdltiHOoQPNsQ==', 'wimpy-chimp-rVPLReCqhEa0s2SSaDNayA==', 'brumal-shrew-ozZe71Zgxf.i0YTk6cEVBg==', 'wilted-lemur-WoqjmugXcCMsRpzu9JLGnA==', 'chic-sow-IuGFC+VorJAAjEwJnIslVA==', 'wimpy-mamba-osnEJkcmUM4yNA2rVLDjNw==', 'booted-budgie-cXP5KjS7cuIqFDfD9tNAmg==', 'pert-shrimp-VhRbkKcpUxFJ08TqHChlIQ==', 'pink-vixen-GumUAE0fmt+DzImDSBhCYw==', 'antic-vermin-7czWp4FcnfJ1CHBPw1ImhA==', 'pumped-rabbit-HtZRgCQrOGVecJzMZif3Tg==', 'velar-syphon-VHjrn8ZUq80vEEj2wboTvA==', 'roan-lynx-JoQAzKq7YHi.RMuQrXETHA==', 'earned-medusa-f3PbMxrmfFf3LUj5cOv0Qw==', 'rushy-sloth-X9iZCJ+YNY684rsT4FKZRQ==', 'awned-toucan-tc5TiuMzf1sXTn98eAZeeg==', 'achy-maggot-pUagPPQhPxagFWJNfY8UzQ==', 'crass-fowl-2oLhFxx+Ejbtb4Rcmtj+sQ==', 'spry-falcon-n94ConbJeDhkS5FwJDFp0w==', 'wigged-bear-lpugt0rZOCjpl3.wisWLjQ==', 'blotto-prawn-gpwZJ4r9FTBY+eZWVM2vyw==', 'amiss-camel-Bbi25Lv8b9+hcyYQ+YiK2g==', 'biform-lion-JR+IAUtWerqPAVi2I7lKSg==', 'algid-deer-HI1n44D7JP4jmvLUsn+Fww==', 'bushed-dayfly-SXVJ6395n.0QLy.wlZKAgA==', 'brunette-koala-m.CAqO.maFbM3Q3w.gZfUg==', 'biped-bison-+G6b0JTznu7IzbwnI2wnWA==', 'joking-raven-KHE4kAqQj9hfn2Sy2HQy9Q==', 'ritzy-donkey-yvvnyj+iJROGKfrXTW6kGg==', 'jowly-colt-rJs1ALgpG5UcE3hjOGEJgg==', 'abulic-badger-KAZpb1eUtchtwAZ1hmLm2w==', 'smudgy-bovid-T13ekKZGEKzvXsk8hL4Vrw==', 'gonzo-parrot-YjZ19WDveFjzWGUF1Bb4AQ==', 'akimbo-rook-pXyXtIWPJitAP2i8DF7eJQ==', 'wound-buck-sAg5yviYulxt2MXldlNy+g==', 'briny-sponge-JJPJEEMRa0SygRSUi9V7lQ==', 'campy-biped-XacbslqqWf.O3XcMmGxnLQ==', 'dashed-doe-Jf5Fmd7QKupLmJVIWWOQhg==', 'corny-filly-nOD+G0MZYQjDIpKiXmvAcg==', 'rose-lemur-A+M9T6U9Y5HJ0NMbtt42DA==', 'focal-platypus-.j1jdWRT4zCxDuT1lQg5vw==', 'faced-cod-dMn2wiN5zxb6h6AFOAyWFQ==', 'smug-beetle-JQI37XLxHVUJmrx..+CPvw==', 'droll-owl-M7djH95vLbUcopwAFr4jng==', 'bulky-kudu-lJ2tIa9mrZK7eNOJSAJq3g==', 'khaki-egg-u0nUs.qiczfOsyvqBsSQ3g==', 'each-cougar-yjZr7xlmuyJpGth31j2j4A==', 'tangy-puma-IH3skSeM4XczmaRQDzf3dA==', 'four-heron-tJ0s1HI32av+byinEHpOpA==', 'sunken-craw-rbH8YWm3Piz78SBhOw71kg==', 'staple-ocelot-xM4aWuhVDgBnrh6VEF0dhw==', 'unary-gerbil-6wVQ2D83eQu3X4LBc4SCNQ==', 'briny-pika-WFjtJ5G.nrOgj3FNfVCEhQ==', 'azoic-canary-C.paIEBPfw8uBRcMSIHn9Q==', 'brumal-skunk-UprOd2A61on2BciB8SR16w==', 'spiky-shrimp-zdakWNkf20wul0NP.5sgrw==', 'nosey-bonobo-gXHFZ+Rr2BE2htAuiT0gVw==', 'equipt-remora-gEr4Da5bGOhR9oerKybyJg==', 'crabby-vervet-3w.Q8ffJ7O1ihKznKwtr7Q==', 'upbeat-zebra-1HdiPEZFKVXMddhpaxi1IQ==', 'sewn-tapir-vLy+gTpbRdj1Et6jcd7n2Q==', 'mystic-pigeon-9+XUv8KyV2eP8J7DvFUXLg==', 'dismal-puffin-di8UbIm1+Ra.Y5TqJtqlVg==', 'slimed-pigeon-2tYIVkGbJGivn2KwnFtKuw==', 'bowing-skate-OKoH6hF+8AJuIkukmoulMw==', 'won-narwal-8NL3L67PhejgTI7ONC78+Q==', 'rotted-krill-kUVcjTIQYefzJr3f67PfVw==', 'loved-eland-C23yCztZ3trQAVK0nXuRnA==', 'scant-biped-+ui9LXUXfwB8k3pUwXKQIA==', 'sly-spider-c7cOcj1WIg9vH6.pwjrOFw==', 'alary-earwig-Ej8VaGFkXIcJGLndw30.2w==', 'agone-ermine-+dz2pyWPTkN5uVNNCuPdbQ==', 'argent-setter-ZopEkXa5V0Jfp3WIOas95A==', 'eldest-alpaca-RWXTNbVsaMRaNeYrtEnv8g==', 'urgent-mink-CkJSlcVzxTW5ouua.ECvWQ==', 'pilous-wrasse-n7h4EheELLNxjZwfzKCqYw==', 'jural-iguana-XVSPHnT8bzjJUZjXPGsvxw==', 'arcane-snail-Y8fifSUplC0mOu5328Uyaw==', 'carven-conger-Fkxe7Pmnv710qhWkoWUfIQ==', 'about-bonobo-.z8rlVc9Jeau7+Sl360sRg==', 'unborn-alpaca-fjTrj92wPzfAbSw8BarMpw==']

        return True

    def list_waypoints(self, upload_path):
        request = ListGraph.Request()
        request.upload_filepath = upload_path
        response = self._waypoint_client.call(request)
        if len(response.waypoint_ids) > 0:
            self._logger.info("ListGraph successful")
            return response.waypoint_ids
        else:
            self._logger.error("ListGraph failed: " + str(response.waypoint_ids))
            return None
        

    def initialize_graph_nav(self, args):
        request = GraphNavInitialize.Request()
        request.upload_filepath = args.upload_path
        request.initial_localization_fiducial = args.initial_localization_fiducial  
        request.initial_localization_waypoint = args.initial_localization_waypoint

        # Use the node
        response = self._grap_nav_client.call(request)

        if response.success:
            self._logger.info("GraphNavInitialize successful")
        else:
            self._logger.error("GraphNavInitialize failed: " + response.message)
            return False        
        return True

    def navigate_to_waypoint(self, args):
        self._logger.info("Navigate to waypoint")
        goal_msg = NavigateTo.Goal()
        goal_msg.navigate_to = self.waypoint_index[args.navigate_to]
        self._robot_navigation_client.send_goal_and_wait("navigate_to", goal_msg)
       
        self._logger.info("Sent goal")

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    parser.add_argument("--navigate_to", type=int, default=0)
    parser.add_argument("--initialize_position", type=bool, default=False)
    parser.add_argument("--upload_path", type=str, default="/home/robot/spot_map/downloaded_graph")
    parser.add_argument("--initial_localization_fiducial", type=bool, default=True)
    parser.add_argument("--initial_localization_waypoint", type=str, default="")
    return parser

@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    navigate_to_waypoint = NavigateToWaypoint(args.robot, main.node)
    navigate_to_waypoint.initialize_robot(args)
    if navigate_to_waypoint.waypoint_index is None:
        return 0
    time.sleep(2)
    position_initialized = True
    print(args.initialize_position)
    if args.initialize_position:
        position_initialized = navigate_to_waypoint.initialize_graph_nav(args)

    if position_initialized:
        navigate_to_waypoint.navigate_to_waypoint(args)
    return 0


if __name__ == "__main__":
    exit(main())
