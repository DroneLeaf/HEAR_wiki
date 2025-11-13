import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: "Software Stack Documentation",
  description: "This documentation is the master entry point for all things DroneLeaf.  ",
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    nav: [
      { text: 'Home', link: '/' },
      { text: 'README', link: '/README.html' }
    ],

    sidebar: [
      {
        text: 'Overview',
        items: [
          { text: 'Home', link: '/' },
          { text: 'Anatomy of the software stack', link: '/README.html#anatomy-of-the-software-stack' },
          { text: 'Definitions of used terms', link: '/README.html#definitions-of-used-terms' }
        ]
      },
      {
        text: 'Getting Started',
        items: [
          { text: 'Deployment', link: '/README.html#getting-started-with-deployment-targeted-for-droneleaf-clients' },
          { text: 'Development', link: '/README.html#getting-started-with-development' },
          { text: 'Preparing the Developer Machine', link: '/README.html#preparing-the-developer-machine' }
        ]
      },
      {
        text: 'Development Stacks',
        items: [
          { text: 'Flight stack', link: '/README.html#getting-started-with-flight-stack-development' },
          { text: 'Petals stack', link: '/README.html#getting-started-with-petals-stack-development' },
          { text: 'Controller Dashboard', link: '/README.html#getting-started-with-controller-dashboard-development' },
          { text: 'Web Client app', link: '/README.html#getting-started-with-web-client-application-development' }
        ]
      },
      {
        text: 'Operations',
        items: [
          { text: 'Commissioning', link: '/README.html#commissioning' },
          { text: 'Running the SITL/bench environment', link: '/README.html#running-the-sitlbench-environment' }
        ]
      },
      {
        text: 'Tools & Support',
        items: [
          { text: 'Debugging Tools', link: '/README.html#debugging-tools' },
          { text: 'Additional Functionalities', link: '/README.html#additional-functionalities' },
          { text: 'Contribution and development', link: '/README.html#contribution-and-development' },
          { text: 'Known Issues', link: '/README.html#known-issues' }
        ]
      }
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/DroneLeaf' }
    ]
  }
})
