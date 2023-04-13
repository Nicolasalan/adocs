export default {
     title: "Docs",
     description: "An awesome docs template built by me",
   
     themeConfig: {
       logo: "/logo.svg",
       siteTitle: "Docs",
       // Navbar Link
       nav: [
         { text: "About", link: "/about" },
         { text: "Contact", link: "/contact" },
         { text: "Guide", link: "/guide" },
         { text: "Configs", link: "/configs" },
       ],
       // Social Icons
       socialLinks: [
         { icon: "github", link: "https://github.com/Nicolasalan" },
         {
           icon: {
             svg: '<svg xmlns="http://www.w3.org/2000/svg" width="1em" height="1em" viewBox="0 0 256 256"><g fill="none"><rect width="256" height="256" fill="#fff" rx="60"></rect><rect width="256" height="256" fill="#0A66C2" rx="60"></rect><path fill="#fff" d="M184.715 217.685h29.27a4 4 0 0 0 4-3.999l.015-61.842c0-32.323-6.965-57.168-44.738-57.168c-14.359-.534-27.9 6.868-35.207 19.228a.32.32 0 0 1-.595-.161V101.66a4 4 0 0 0-4-4h-27.777a4 4 0 0 0-4 4v112.02a4 4 0 0 0 4 4h29.268a4 4 0 0 0 4-4v-55.373c0-15.657 2.97-30.82 22.381-30.82c19.135 0 19.383 17.916 19.383 31.834v54.364a4 4 0 0 0 4 4ZM38 59.627c0 11.865 9.767 21.627 21.632 21.627c11.862-.001 21.623-9.769 21.623-21.631C81.253 47.761 71.491 38 59.628 38C47.762 38 38 47.763 38 59.627Zm6.959 158.058h29.307a4 4 0 0 0 4-4V101.66a4 4 0 0 0-4-4H44.959a4 4 0 0 0-4 4v112.025a4 4 0 0 0 4 4Z"></path></g></svg>',
           },
           link: "https://www.linkedin.com/in/nicolas-alan-grotti/",
         },
       ],
       // Sidebar
       sidebar: [
         {
           text: "Navigation",
           collapsible: true,
           items: [
             { text: "Introduction Navigation Nav2", link: "/navigation/introduction" },
             { text: "How to Build a Map", link: "/navigation/mapping" },
             { text: "How to Localize the Robot in the Environment", link: "/navigation/localization" },
             { text: "How to do Path Planning", link: "/navigation/path-planning" },
             { text: "How Obstacle Avoidance Happens in ROS", link: "/navigation/obstacle" },
           ],
         },
         {
          text: "Advanced ROS2 Navigation",
          collapsible: true,
          items: [
            { text: "New Nav2 Features", link: "/advanced/new" },
            { text: "Behavior Trees", link: "/advanced/behavior" },
            { text: "Nav2 Plugins", link: "/advanced/plugins" },
            { text: "Controller Server In Deep", link: "/advanced/deep" },
          ],
        },
        {
          text: "Behavior Trees",
          collapsible: true,
          items: [
            { text: "Introduction to Behavior Trees", link: "/Behavior_Trees/introduction" },
            { text: "Design Principles", link: "/Behavior_Trees/design" },
            { text: "Behavior with ROS 2", link: "/Behavior_Trees/ros2" },
            { text: "Stochastic Behavior Trees", link: "/Behavior_Trees/stochastic" },
          ],
        },
       ],
       footer: {
         message: "Released under the MIT License.",
         copyright: "Copyright © 2022-present robot-docs",
       },
       markdown: {
         theme: "material-palenight",
         lineNumbers: true,
       },
     },
   };
   