"""
Exports Issues from a specified repository to a CSV file
Uses basic authentication (Github username + password) to retrieve Issues
from a repository that username has access to. Supports Github API v3.
"""
import csv
import requests


GITHUB_USER = 'zimmer1'
GITHUB_PASSWORD = 'booboo809'
REPO = 'zimmer1/smarda'  # format is username/repo
ISSUES_FOR_REPO_URL = 'https://api.github.com/repos/%s/issues' % REPO
AUTH = (GITHUB_USER, GITHUB_PASSWORD)


def get_issues(response):
    issues = []


    "output a list of issues to a list"
    if not r.status_code == 200:
        raise Exception(r.status_code)
        print("exception raised")
        return
    for issue in r.json():
        labels = issue['labels']
        for label in labels:
            if label['name'] == "Client Requested":
                issues.append([issue['number'], issue['title'].encode('utf-8'), issue['body'].encode('utf-8'), issue['created_at'], issue['updated_at']])


    return issues_list

def getGit():
    r = requests.get(ISSUES_FOR_REPO_URL, auth=AUTH)

    print("Requests happened")

    #more pages? examine the 'link' header returned
    if 'link' in r.headers:
        pages = dict(
            [(rel[6:-1], url[url.index('<')+1:-1]) for url, rel in
             [link.split(';') for link in
              r.headers['link'].split(',')]])
        while 'last' in pages and 'next' in pages:
            r = requests.get(pages['next'], auth=AUTH)
            issue_list = get_issues(r)
            print(issue_list)
            if pages['next'] == pages['last']:
                break


getGit()
